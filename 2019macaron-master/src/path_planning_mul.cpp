#include "ros/ros.h"
#include <iostream>
#include "math.h"

#include "sensor_msgs/LaserScan.h"
#include "macaron/Floats.h"
#include "macaron/base_frame.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int32MultiArray.h"

#define PI acos(-1)

//for visualization
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
visualization_msgs::Marker      path_rviz_msg;
visualization_msgs::MarkerArray path_viewer;
visualization_msgs::Marker      obstacle_rviz_msg;
visualization_msgs::MarkerArray obstacle_viewer;
visualization_msgs::Marker      lane_rviz_msg;
visualization_msgs::MarkerArray lane_viewer;

#define _K_city_x 515540
#define _K_city_y 179847
#define _8line_x 550922
#define _8line_y 199952
#define _pankyu_x 533412
#define _pankyu_y 209081

const double offset_x = _pankyu_x;
const double offset_y = _pankyu_y;

#define DEG2RAD  PI/180


//-----------------------------------------------------------------------//
//  Current Coordinate, Every single angle starts from X axis to Y axis  //
//                                                                       //
//                         X                                             //
//                         ^                                             //
//                         |                                             //
//                         |                                             //
//                         |                                             //
//                         |                                             //
//                         |                                             //
//                         |                                             //
//                         0 ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ> Y                       //
//-----------------------------------------------------------------------//


//----------------control property---------------------------//
double wheel_base = 1.040, tread = 0.985, width = 1.160; //macaron property
double lidar_offset = 0;         //라이다가 전방에서 삐뚤어진 각도, rad, 반시계가 +값
double d_lidar_gps = 1;          //라이다와 gps사이의 거리.
#define candidate_num 6          //중심 제외 단방향으로의 후보경로       
double search_range  = 1;        //단방향 후보경로 탐색범위(m) ,q방향
#define candidate_path_leng 10   //후보경로의 절점개수
double path_leng = 7;            //후보경로의 길이 s길이 (m)
double lane_width = 3.5;         //차선 변경이 필요한 구간의 차선 폭, 예선 3.7, 본선 3.5
double obstacle_checking_d = 15; //장애물을 탐지하기 시작하는 거리

double w_offset      = 0.5;
double w_lane        = 0.0; 
double w_consistency = 0.1;
double w_obstacle    = 50.0;

double cost_offset;
double cost_lane;
double cost_consistency;
double cost_obstacle;
//-------------------input value--------------------------//
//from lanetraker
double lane_data[2][23];
double lane_abs[2][23];  //차선 중심의 절대좌표
int lane_data_size;      //lane_data 열의 크기
int seq_before = 0;
int lane_error = 1;

//from lidar
double scan_data[541] = { 0 };        //극좌표계(상대좌표계), 캘리브레이션 안되어있음
double obstacle_data[3][541] = { 0 }; //xy좌표계(절대좌표계), 캘리브레이션 되어있음. 3번째 행에는 위험구역 내의 장애물인지 아닌지에 대한 정보가 저장.(1이 위험)

//from gps_txt reader
double base_path[2][int(candidate_path_leng)]      = { 0 };
double base_path_vec[int(candidate_path_leng)]     = { 0 };
double candidate_path[2][int(candidate_path_leng)] = { 0 };

double localized_p_before[2];   
double qi;                       //초기 오프셋
double qf = 0;                   //목표 오프셋
double qf_before;                //이전 목표 오프셋

//from Localization node
double x_tm;
double y_tm;
double heading; //도북기준 heading, rad

//from mission identifier
int available_path[3] = {0, 1, 0};
int mission_num;

//-------------------write value--------------------------//
std_msgs::Float64MultiArray selected_path;
std_msgs::Int32 e_stop;
//-----------------ETC global value------------------------//
int selected_index;
int static_obstacle[2][2];
int selected_lane;
int safe_lane             = 1; //안전한 차선, 왼쪽부터 0 1 2 (1이 기본 차선)
int dynamic_obstacle      = 0; //동적 장애물 등장 여부
int memorized_safe_line   = 1;
int count_for_lane_change = 0;
int lane_changing_time    = 5; //sec

#define dy_obs_mission 1
#define parking_mission 2


void headingCallBack(const geometry_msgs::Vector3::ConstPtr& location)
{
    x_tm    = location->x;
    y_tm    = location->y;
    heading = location->z; //rad
}


void laneCallBack(const macaron::Floats::ConstPtr& lane)
{
    int seq        = lane->header.seq;
    int dstride0   = lane->testset.layout.dim[0].stride;
    int dstride1   = lane->testset.layout.dim[1].stride;
    int h          = lane->testset.layout.dim[0].size;
    lane_data_size = lane->testset.layout.dim[1].size;
    for (int i = 0; i < h; i++)
    {
        for (int j = 0; j < lane_data_size; j++)
        {
            lane_data[i][j] = lane->testset.data[dstride1*i+j];
        }
    }
    for(int i = 0; i < lane_data_size; i++)
    {
        lane_abs[0][i] = x_tm + lane_data[0][i]*cos(heading) + lane_data[1][i]*cos(heading + PI/2); //deviation은 내가 차선 오른쪽에 있을 때 + 이다.
        lane_abs[1][i] = y_tm + lane_data[0][i]*sin(heading) + lane_data[1][i]*sin(heading + PI/2);
    }
    if(seq == seq_before)
        lane_error = 1;
    else
        lane_error = 0;
    seq_before = seq;
    //차선을 제대로 읽지 못하고 흔들릴 때 이를 보정하는 코드가 필요, 퍼블리시 안할 때 seq 동일한지 확인 필요
}


void scanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    for(int i = 0; i < 541; i++)
    {
        scan_data[i] = scan->ranges[i + 135]; 
        if(isinff(scan_data[i]) || scan_data[i] < 0.5 || scan_data[i] > 20) //측정값을 라이다의 측정가능 거리내로 필터링
            scan_data[i] = 25;
        double theta = (double(i) / 3.0 - 90) * PI / 180.0; //라이다 기준 실제 각도, rad
        obstacle_data[0][i] = x_tm + d_lidar_gps*cos(heading) + scan_data[i]*cos(heading - lidar_offset - theta);
        obstacle_data[1][i] = y_tm + d_lidar_gps*sin(heading) + scan_data[i]*sin(heading - lidar_offset - theta);
        obstacle_data[2][i] = 0;
    }
}


void pathCallBack(const macaron::base_frame::ConstPtr& path) 
{
    printf("Subscring base path from GPS TXT reader\n");
    localized_p_before[0] = base_path[0][0];
    localized_p_before[1] = base_path[1][0];
    qi                    = path->distance;
    path_leng             = path->ld;
    for(int i = 0; i < int(candidate_path_leng); i++)
    {
        base_path[0][i]  = path->s_x[i];
        base_path[1][i]  = path->s_y[i];
        base_path_vec[i] = path->s_a[i];
    }
}


void ablepathCallBack(const::std_msgs::Int32MultiArray::ConstPtr& lane_change)
{
    for(int i = 0; i < 3; i++)
    {
        available_path[i] = lane_change->data[i];
    }
}


void missionCallBack(const::std_msgs::Int32::ConstPtr& mission)
{
    mission_num = mission->data;
}


void generate_candidate_path(int index, int lane)
{
    qf = search_range / double(candidate_num) * double(index) + (1 - lane) * lane_width;
    double theta = heading - base_path_vec[0];
    double ds = path_leng;
    double a1 = (ds * tan(theta) - 2 * (qf - qi)) / (ds * ds * ds);
    double a2 = (qf - qi) / (ds * ds);

    for(int i = 0; i < int(candidate_path_leng); i++)
    {
        double s = path_leng / double(candidate_path_leng - 1) * i;
        double offset = (s - ds)*(s - ds) * (a1*s - a2) + qf;
        candidate_path[0][i] = base_path[0][i] + offset * cos(base_path_vec[i] - PI/2);
        candidate_path[1][i] = base_path[1][i] + offset * sin(base_path_vec[i] - PI/2);
    }

    selected_path.data.resize(int(candidate_path_leng) * 2);
    for(int xy = 0; xy < 2; xy++)
    {
        for(int i = 0; i < int(candidate_path_leng); i++)
        {
             selected_path.data[i + int(candidate_path_leng) * xy] = candidate_path[xy][i];
        }
    }
}


void checking_danger_lane()
{
    double obstacle_d_lane1 = obstacle_checking_d;
    double obstacle_d_lane2 = obstacle_checking_d;
    for(int i = 0; i < 541; i++)
    {
        //기본경로 기반으로 변경된 장애물좌표
        double x = +cos(base_path_vec[0])*obstacle_data[0][i] + sin(base_path_vec[0])*obstacle_data[1][i] - base_path[0][0]*cos(base_path_vec[0]) - base_path[1][0]*sin(base_path_vec[0]);
        double y = -sin(base_path_vec[0])*obstacle_data[0][i] + cos(base_path_vec[0])*obstacle_data[1][i] + base_path[0][0]*sin(base_path_vec[0]) - base_path[1][0]*cos(base_path_vec[0]);
        //기본 차선. 돌방 장애물 등장구역에선 돌발장애물을 판별한다.
        if (x < obstacle_checking_d && x > 0 && fabs(y) < search_range / 2.0)
        {
            if(mission_num == dy_obs_mission)
                dynamic_obstacle = 1;
            obstacle_data[2][i] = 1;
            double TEMP = x;
            if(TEMP < obstacle_d_lane1)
            {
                obstacle_d_lane1 = TEMP;                    
            }
        }
        //변경할 차선. 한쪽으로만 차선을 바꿀 수 있다고 가정. 동적장애물 등장 구역에선 사용하지 않음.
        else if (x < obstacle_checking_d && x > 0 && fabs(y + lane_width*(available_path[0] - available_path[2])) < search_range / 2.0 && !(mission_num == dy_obs_mission))
        {
            obstacle_data[2][i] = 1;
            double TEMP = x;
            if(TEMP < obstacle_d_lane2)
            {
                obstacle_d_lane2 = TEMP;                    
            }
        }
    }
    if(obstacle_d_lane1 < obstacle_d_lane2)
    {
        safe_lane = 1 + (available_path[2] - available_path[0]);
    } 
    else
    {
        safe_lane = 1;
    }
}


double path_cost(int lane)
{
    //offset cost
	cost_offset = fabs(qf);

	//lane cost, if data set have an error, ignore it.
    double lane_d = 100; //임의의 큰 수, 후보경로의 끝점으로 부터 차선중심까지의 거리
    for(int i = 0; i < lane_data_size; i++)
    {
        double TEMP_d = sqrt(pow(candidate_path[0][int(candidate_path_leng) - 1] - lane_abs[0][i], 2) + pow(candidate_path[1][int(candidate_path_leng) - 1] - lane_abs[1][i], 2));
        if(TEMP_d < lane_d)
            lane_d = TEMP_d;
    }
    cost_lane = lane_d * (1 - lane_error);

	//consistency cost
	double common_s = path_leng - sqrt(pow(base_path[0][0] - localized_p_before[0], 2) + pow(base_path[1][0] - localized_p_before[1], 2));
	cost_consistency = fabs(qf - qf_before) * common_s * 0.5;

    //obstacle cost
    if(lane == safe_lane)
        cost_obstacle = 0;
    else
        cost_obstacle = 1;

	return w_offset*cost_offset + w_lane*cost_lane + w_consistency*cost_consistency + w_obstacle*cost_obstacle;	
}


void print_on_terminal()
{   
    if(selected_lane == 0)
        printf("Selected Path Index       : Left lane\n");
    else if(selected_lane == 1)
        printf("Selected Path Index       : Middle lane\n");
    else if(selected_lane == 2)
        printf("Selected Path Index       : Right lane\n");
    printf("Mission Number            : %d\n",mission_num);
    printf("Selected Path Index       : %d\n",selected_index);
    printf("Selected Offset from GPS  : %fm\n\n",qf);
    printf("Selected Offset cost      : %f\n",cost_offset);
    printf("Selected Lane cost        : %f\n",cost_lane);
    printf("Selected Consistency cost : %f\n",cost_consistency);
    printf("Selected Obstacle cost    : %f\n",cost_obstacle);
    printf("---------------------------------------------------\n");
}


void visualizing_path(int index, int lane, int selected)
{
    for(int i = 0; i < int(candidate_path_leng); i++)
    {
        geometry_msgs::Point p;
        std_msgs::ColorRGBA c;
        c.a = 1.0; c.r = 1.0; c.g = 1.0; c.b = 1.0 - 1 * selected;
        p.x = candidate_path[0][i] - offset_x;
        p.y = candidate_path[1][i] - offset_y;
        p.z = 0.0;
        path_rviz_msg.type = visualization_msgs::Marker::SPHERE;    
        path_rviz_msg.scale.x = 0.3 + 0.2 * selected;
        path_rviz_msg.scale.y = 0.3 + 0.2 * selected;
        path_rviz_msg.scale.z = 0.1;
        path_rviz_msg.pose.position   = p;
        path_rviz_msg.color           = c;
        path_rviz_msg.header.frame_id = "map";
        path_rviz_msg.action = visualization_msgs::Marker::ADD;
        path_rviz_msg.id = i + 100 + (index + candidate_num) * int(candidate_path_leng) + lane * (2 * candidate_num + 1) * int(candidate_path_leng);
        path_viewer.markers.push_back(path_rviz_msg);
    }
}


void visualizing_obstacle()
{
    for(int i = 0; i < 541; i++)
    {
        geometry_msgs::Point p;
        std_msgs::ColorRGBA c;
        c.a = 0.5; c.r = 1.0; c.g = 0.5; c.b = 0.0;
        p.x = obstacle_data[0][i] - offset_x;
        p.y = obstacle_data[1][i] - offset_y;
        p.z = 0.0;
        obstacle_rviz_msg.type = visualization_msgs::Marker::SPHERE;    
        obstacle_rviz_msg.scale.x = 0.2;
        obstacle_rviz_msg.scale.y = 0.2;
        obstacle_rviz_msg.scale.z = 0.1;
        obstacle_rviz_msg.pose.position   = p;
        obstacle_rviz_msg.color           = c;
        obstacle_rviz_msg.header.frame_id = "map";
        obstacle_rviz_msg.action = visualization_msgs::Marker::ADD;
        obstacle_rviz_msg.id = i + 500;
        obstacle_viewer.markers.push_back(obstacle_rviz_msg);  
    }
    for(int i = 0; i < 541; i++)
    {   
        geometry_msgs::Point p;
        std_msgs::ColorRGBA c;
        c.a = 0.0; c.r = 1.0; c.g = 0.0; c.b = 0.0;
        if(obstacle_data[2][i] == 1)
            c.a = 1.0;
        p.x = obstacle_data[0][i] - offset_x;
        p.y = obstacle_data[1][i] - offset_y;
        p.z = 0.0;
        obstacle_rviz_msg.type = visualization_msgs::Marker::SPHERE;    
        obstacle_rviz_msg.scale.x = 0.3;
        obstacle_rviz_msg.scale.y = 0.3;
        obstacle_rviz_msg.scale.z = 0.1;
        obstacle_rviz_msg.pose.position   = p;
        obstacle_rviz_msg.color           = c;
        obstacle_rviz_msg.header.frame_id = "map";
        obstacle_rviz_msg.action = visualization_msgs::Marker::ADD;
        obstacle_rviz_msg.id = i + 1041;
        obstacle_viewer.markers.push_back(obstacle_rviz_msg);
    }
}


void visualizing_lane()
{
    for(int i = 0; i < lane_data_size; i++)
    {
        geometry_msgs::Point p;
        std_msgs::ColorRGBA c;
        c.a = 1.0; c.r = 1.0; c.g = 0.0; c.b = 1.0;
        p.x = lane_abs[0][i] - offset_x;
        p.y = lane_abs[1][i] - offset_y;
        p.z = 0.0;
        lane_rviz_msg.type = visualization_msgs::Marker::SPHERE;    
        lane_rviz_msg.scale.x = 0.6;
        lane_rviz_msg.scale.y = 0.6;
        lane_rviz_msg.scale.z = 0.1;
        lane_rviz_msg.pose.position   = p;
        lane_rviz_msg.color           = c ;
        lane_rviz_msg.header.frame_id = "map";
        lane_rviz_msg.action = visualization_msgs::Marker::ADD;
        lane_rviz_msg.id=i + 2000;
        lane_viewer.markers.push_back(lane_rviz_msg);
    }
}


int main(int argc, char **argv)
{
    ros::init(argc,argv,"path_planning");
    ros::NodeHandle nh;
    ros::Rate loop_rate(50);

    ros::Publisher path_planner        = nh.advertise<std_msgs::Float64MultiArray>("/selected_path", 10);
    ros::Publisher E_stop_pub          = nh.advertise<std_msgs::Int32>("/E_stop", 10);
    ros::Publisher path_viewer_pub     = nh.advertise<visualization_msgs::MarkerArray>("/path_test",10);
    ros::Publisher obstacle_viewer_pub = nh.advertise<visualization_msgs::MarkerArray>("/obstacle",10);
    ros::Publisher lane_viewer_pub     = nh.advertise<visualization_msgs::MarkerArray>("/lane_viewer",10);

    ros::Subscriber localization       = nh.subscribe("/heading",10,headingCallBack);
    ros::Subscriber lane_sub           = nh.subscribe("/lane",10, laneCallBack);
    ros::Subscriber gpspath_sub        = nh.subscribe("/base_frame",10, pathCallBack);
    ros::Subscriber available_path_sub = nh.subscribe("/available_path", 10, ablepathCallBack);
    ros::Subscriber lidar_sub          = nh.subscribe("/scan", 10, scanCallBack);
    ros::Subscriber mission_num_sub    = nh.subscribe("/mission_num", 10, missionCallBack);

    while(ros::ok)
    {
        ros::spinOnce();

        double cost_TEMP = 100; //임의의 큰 수
        safe_lane = 1;          //안전한 차선 번호
        dynamic_obstacle = 0;   //돌발 장애물 등장 여부
        
        if(available_path[0] + available_path[2]) //정적 장애물 등장 구역이면,
            checking_danger_lane();

        if(mission_num == dy_obs_mission)         //돌발 장애물 등장 구역이면,
            checking_danger_lane();   
        e_stop.data = dynamic_obstacle;
        if(dynamic_obstacle)
            printf("stop\n");

        for(int i = 0; i < 3; i++)
        {
            if(available_path[i])
            {
                for(int index = -candidate_num; index <= candidate_num; index++)
                {
                    generate_candidate_path(index, i);
                    visualizing_path(index, i, 0);
                    if(path_cost(i) < cost_TEMP)
                    {
                        cost_TEMP = path_cost(i);
                        selected_index = index;
                        selected_lane = i;
                    }
                }
            }
        }
        generate_candidate_path(selected_index, selected_lane);
        path_cost(selected_lane);
        visualizing_path(candidate_num + 1, 2, 1);
        qf_before = qf;
        print_on_terminal();
        visualizing_obstacle();
        visualizing_lane();
        path_viewer_pub.publish(path_viewer);
        path_viewer.markers.clear();
        obstacle_viewer_pub.publish(obstacle_viewer);
        obstacle_viewer.markers.clear();
        if(!lane_error)
        {
            lane_viewer_pub.publish(lane_viewer);
            lane_viewer.markers.clear();
        }
        E_stop_pub.publish(e_stop);
        path_planner.publish(selected_path);
        loop_rate.sleep();
    }
    return 0;
}
