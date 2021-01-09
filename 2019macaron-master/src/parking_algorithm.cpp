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
visualization_msgs::Marker      lane_rviz_msg;
visualization_msgs::MarkerArray lane_viewer;
visualization_msgs::Marker      obstacle_rviz_msg;
visualization_msgs::MarkerArray obstacle_viewer;

#define _K_city_x 515540
#define _K_city_y 179847
#define _8line_x 550922
#define _8line_y 199952

const double offset_x=_K_city_x;
const double offset_y=_K_city_y;


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
#define candidate_path_leng 10   //후보경로의 절점개수
//-------------------input value--------------------------//

//from lidar
double scan_data[541] = { 0 };        //극좌표계(상대좌표계), 캘리브레이션 안되어있음
double obstacle_data[3][541] = { 0 }; //xy좌표계(절대좌표계), 캘리브레이션 되어있음. 3번째 행에는 위험구역 내의 장애물인지 아닌지에 대한 정보가 저장.(1이 위험)

//from Localization node
double x_tm;
double y_tm;
double heading; //도북기준 heading, rad

//-------------------write value--------------------------//
std_msgs::Float64MultiArray selected_path;
std_msgs::Int32 e_stop;
//-----------------ETC global value------------------------//
double parking_x[6]        = {515598.22, 515600.86, 515603.12, 515605.59, 515608.36, 515610.73};
double parking_y[6]        = {179882.45, 179884.12, 179885.29, 179886.43, 179888.12, 179890.83};
double base_path_vec       = 0.528342914882; //직선구간 기울기, rad
double parking_area_length = 1.5;
double parking_area_width  = 3.0;

void headingCallBack(const geometry_msgs::Vector3::ConstPtr& location)
{
    x_tm     = location->x;
    y_tm     = location->y;
    heading  = location->z; //rad
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


void checking_parking_area(int parking_index)
{
    for(int i = 0; i < 541; i++)
    {
        //주차위치 기반으로 변경된 장애물좌표
        double theta = base_path_vec + 40 * PI / 180.0;
        double x = +cos(theta)*obstacle_data[0][i] + sin(theta)*obstacle_data[1][i] - parking_x[parking_index]*cos(theta) - parking_y[parking_index]*sin(theta);
        double y = -sin(theta)*obstacle_data[0][i] + cos(theta)*obstacle_data[1][i] + parking_x[parking_index]*sin(theta) - parking_y[parking_index]*cos(theta);
        if (fabs(x) < parking_area_length / 2.0 && fabs(y) < parking_area_width / 2.0)
        {
            obstacle_data[2][i] = 1;
        }
    }
}


void visualizing_lane()
{
    for(int i = 0; i < 6; i++)
    {
        geometry_msgs::Point p;
        std_msgs::ColorRGBA c;
        c.a = 0.3; c.r = 1.0; c.g = 0.0; c.b = 1.0;
        p.x = parking_x[0] - offset_x;
        p.y = parking_y[0] - offset_y;
        p.z = 0.0;
        lane_rviz_msg.type = visualization_msgs::Marker::SPHERE;    
        lane_rviz_msg.scale.x = 0.6;
        lane_rviz_msg.scale.y = 0.6;
        lane_rviz_msg.scale.z = 0.1;
        lane_rviz_msg.pose.position   = p;
        lane_rviz_msg.color           = c ;
        lane_rviz_msg.header.frame_id = "map";
        lane_rviz_msg.action = visualization_msgs::Marker::ADD;
        lane_rviz_msg.id = 12000 + i;
        lane_viewer.markers.push_back(lane_rviz_msg);
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
        obstacle_rviz_msg.id = i + 10500;
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
        obstacle_rviz_msg.id = i + 11041;
        obstacle_viewer.markers.push_back(obstacle_rviz_msg);
    }
}


int main(int argc, char **argv)
{
    ros::init(argc,argv,"parking");
    ros::NodeHandle nh;
    ros::Rate loop_rate(50);

    //ros::Publisher E_stop_pub            = nh.advertise<std_msgs::Int32>("/E_stop", 10);
    ros::Publisher parking_obstacle_pub  = nh.advertise<visualization_msgs::MarkerArray>("/parking_obstacle",10);
    ros::Publisher parking_lane_pub      = nh.advertise<visualization_msgs::MarkerArray>("/parking_lane",10);

    ros::Subscriber parking_localization = nh.subscribe("/heading",10,headingCallBack);
    ros::Subscriber parking_lidar_sub    = nh.subscribe("/scan", 10, scanCallBack);

    while(ros::ok)
    {
        ros::spinOnce();
        for(int i = 0;i < 6;i++)
        {
            checking_parking_area(0);
        }
        visualizing_lane();
        visualizing_obstacle();
        parking_lane_pub.publish(lane_viewer);
        lane_viewer.markers.clear();
        parking_obstacle_pub.publish(obstacle_viewer);
        obstacle_viewer.markers.clear();
        loop_rate.sleep();
    }
    return 0;
}
