//ros basic header file
#include "ros/ros.h"
#include "math.h"
//message header file
#include "geometry_msgs/Vector3.h"
#include "macaron/base_frame.h"
#include "macaron/Floats_for_mission.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float64.h"
#include "darknet_ros_msgs/BoundingBox.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include <parking_algorithm/Message1.h>

//global
int node_index;
int caution_deviation = 25;
int stop_deviation = 5;
float deviation,left_curv, right_curv;
int line_type,error,fair_state;
bool left_trun,right_trun;

bool flag=1;
float traffic_flag[10][4];

float read_right[3]={0.65,0.25,0.1};
float beyes_GRX[3]={0.333333,0.333333,0.333333};
float beyes_TL[2]={0.5,0.5};

int park_seq;

#define dy_obs_mission 1
#define parking_mission 2

darknet_ros_msgs::BoundingBoxes traffic_signs;
std_msgs::Int32MultiArray available_path;
std_msgs::Float64 speed_reduction;
std_msgs::Int32 mission_num;
std_msgs::Int32 crosswalk_e_stop;


void baseCALLBACK(const std_msgs::Int32::ConstPtr& index)
{
    node_index = index->data;
}


void trafficCallBack(const darknet_ros_msgs::BoundingBoxes::ConstPtr& detection)
{
    for(int i = 0 ; i < detection->bounding_boxes.size(); i++){
        traffic_flag[detection->bounding_boxes[i].Class][1] = (detection->bounding_boxes[i].xmin+detection->bounding_boxes[i].xmax)/2;
        traffic_flag[detection->bounding_boxes[i].Class][2] = (detection->bounding_boxes[i].ymin+detection->bounding_boxes[i].ymax)/2;
        traffic_flag[detection->bounding_boxes[i].Class][3] = (-detection->bounding_boxes[i].ymin+detection->bounding_boxes[i].ymax)*(-detection->bounding_boxes[i].xmin+detection->bounding_boxes[i].xmax);

        if((-detection->bounding_boxes[i].xmin+detection->bounding_boxes[i].xmax)>10){
            traffic_flag[detection->bounding_boxes[i].Class][0] = detection->bounding_boxes[i].probability;
        }
    }
}


void laneCallBack(const macaron::Floats_for_mission::ConstPtr& mission)
{
    float deviation=mission->deviation;
    float left_curv=mission->left_curv;
    float right_curv=mission->right_curv;
    int line_type=mission->line_type;
    int error=mission->error;
    int fair_state=mission->fair_state;
    bool left_trun=mission->left_trun;
    bool right_trun=mission->right_trun;
}


void parkCallBack(const parking_algorithm::Message1::ConstPtr& park)
{
    park_seq = park->seq_num;
}


int check_traffic(int signal)
{
    int green;
    if(signal == 0) //직진
    {
        if(beyes_GRX[2]>beyes_GRX[0] && beyes_GRX[1]>beyes_GRX[0])
            green = 1;
        else if(beyes_GRX[0]>beyes_GRX[1])
            green = 1;
        else if(beyes_GRX[0]<beyes_GRX[1])
            green = 0;
    }
    else if(signal == 1) //좌회전
    {
        if(beyes_TL[1]>0.8)
            green = 1;
        else if((beyes_TL[1]<0.2)) 
            green = 0;
    }
    return green;
}


void bayes_traffic()
{
        double G,R,X,a;
        double turn_left,tx;

        if(traffic_flag[9][0]<0.2 && beyes_TL[0]<0.9){
            tx=beyes_TL[0]*0.55;
            turn_left=beyes_TL[1]*0.45;
            beyes_TL[0]=tx/(turn_left+tx);
            beyes_TL[1]=turn_left/(turn_left+tx);

        }
        else if(traffic_flag[9][0]>0.2 && beyes_TL[1]<0.9){
            tx=beyes_TL[0]*0.01;
            turn_left=beyes_TL[1]*0.99;
            beyes_TL[0]=tx/(turn_left+tx);
            beyes_TL[1]=turn_left/(turn_left+tx);
        }


        if(traffic_flag[0][0]<0.3 && traffic_flag[1][0]<0.3){
        G=beyes_GRX[0]*0.4;
        R=beyes_GRX[1]*0.4;
        X=beyes_GRX[2]*0.2;

        beyes_GRX[0]=G/(G+R+X);
        beyes_GRX[1]=R/(G+R+X);
        beyes_GRX[2]=X/(G+R+X);
        a=0;
        }
        else if(traffic_flag[0][0]>traffic_flag[1][0] && beyes_GRX[0]<0.95){
            G=beyes_GRX[0]*read_right[0];
            R=beyes_GRX[1]*read_right[1];
            X=beyes_GRX[2]*read_right[2];

            beyes_GRX[0]=G/(G+R+X);
            beyes_GRX[1]=R/(G+R+X);
            beyes_GRX[2]=X/(G+R+X);
            a=1;

        }
        else if(traffic_flag[0][0]<traffic_flag[1][0] && beyes_GRX[1]<0.95){
            G=beyes_GRX[0]*read_right[1];
            R=beyes_GRX[1]*read_right[0];
            X=beyes_GRX[2]*read_right[2];

            beyes_GRX[0]=G/(G+R+X);
            beyes_GRX[1]=R/(G+R+X);
            beyes_GRX[2]=X/(G+R+X);
            a=2;
        }
    for(int i=0; i<10; i++)
    {
        for(int j=0; j<4; j++)
    {
        traffic_flag[i][j] = 0;
    }
    }
        for(int i=0; i<10; i++){printf("%f  ",traffic_flag[i][0]);}
        printf("\n");
        for(int i=0; i<3; i++){printf("%f ",beyes_GRX[i]);}
        printf("\n");
        for(int i=0; i<2; i++){printf("%f ",beyes_TL[i]);}
        printf("\n------------------------------------------------ %f\n",    speed_reduction.data);
}


void mission_identify()
{
    //기본 초기값
    mission_num.data  = 0;
    speed_reduction.data   = 1.0;
    crosswalk_e_stop.data  = 0;
    available_path.data.resize(3);
    available_path.data[0] = 0; //차선 변경 불가
    available_path.data[1] = 1;
    available_path.data[2] = 0;
    flag = 1;
    /*
    //팔정도 모의 실험
    if(node_index >= 88 && node_index <= 120)      //cross walk
    {
        if(traffic_flag[0] > 0) 
            flag = 1;
        else if(traffic_flag[1] > 0)
            flag = 0;
        speed_reduction.data = flag;
        printf("Mission : Cross walk\n");
        if(flag)
            printf("    Green light\n");
        else
            printf("    Red light\n");  
    }
    else if(node_index >= 140 && node_index <= 174) //safe zone
    {
        speed_reduction.data = 0.7;
        printf("Mission : Safe zone\n");
    }
    else if(node_index >= 191 && node_index <= 226) //dynamic obstacle
    {
        mission_num.data = dy_obs_mission;
        printf("Mission : Dynamic obstacle\n");
    }
    else                                            // normal course
        printf("Mission : Normal course\n");
    printf("\n---------------------------------------------------\n\n");
    */
    
    //만해광장 장애물 회피 실험
    /*
    if(node_index >= 85 && node_index <= 110) //정적 장애물 등장구역
    {
        available_path.data[0] = 0; //오른쪽으로 차선변경
        available_path.data[1] = 1;
        available_path.data[2] = 1;
        printf("Mission : Static obstacle\n");
    }
    else                                            // normal course
        printf("Mission : Normal course\n");
    printf("\n---------------------------------------------------\n\n");
    */
    
    //k-city 예선
    /*
    int stopline[4] = {230, 284, 375, 412};
    if(node_index >= 260 && node_index <= 330)
    {
        speed_reduction.data = 0.7;
        printf("School Zone\n");
    }
    if(node_index >= 124 && node_index <= 151)                           //정적 장애물(드럼 통) 등장 구간
    {
        speed_reduction.data   = 0.5;
        available_path.data[0] = 1; //좌측으로 차선변경 가능
        available_path.data[1] = 1;
        available_path.data[2] = 0;
        printf("Mission : Static obstacle\n");
    }
    else if(node_index >= stopline[0] - caution_deviation && node_index <= stopline[0]) //신호등, 직진
    {
        flag = check_traffic(0);//직진
        printf("Mission : Cross walk\n");
        //파란불
        if(flag)
            printf("    Green light\n");
        //빨간불, 감속    
        else if(stopline[0] - node_index > stop_deviation)
        {
            speed_reduction.data = 0.3;
            printf("    Red light\n");
        }      
        //빨간불, 정지
        else
        {
            crosswalk_e_stop.data = 1;
            printf("    Red light\n");
            printf("     STOP!!!!\n");
        }     
    }
    else if(node_index >= stopline[1] - caution_deviation && node_index <= stopline[1]) //신호등, 좌회전
    {
        flag = check_traffic(1);//좌회전
        printf("Mission : Cross walk\n");
        //파란불
        if(flag)
            printf("    Green light\n");
        //빨간불, 감속    
        else if(stopline[1] - node_index > stop_deviation)
        {
            speed_reduction.data = 0.3;
            printf("    Red light\n");
        }      
        //빨간불, 정지
        else
        {
            crosswalk_e_stop.data = 1;
            printf("    Red light\n");
            printf("     STOP!!!!\n");
        }     
    }
    else if(node_index >= 316 && node_index <= 330)                      //돌발 장애물 등장 구간
    {
        mission_num.data = dy_obs_mission;
        printf("Mission : Dynamic obstacle\n");
    }
    else if(node_index >= stopline[2] - caution_deviation && node_index <= stopline[2]) //신호등, 좌회전
    {
        flag = check_traffic(1);//좌회전
        printf("Mission : Cross walk\n");
        //파란불
        if(flag)
            printf("    Green light\n");
        //빨간불, 감속    
        else if(stopline[2] - node_index > stop_deviation)
        {
            speed_reduction.data = 0.3;
            printf("    Red light\n");
        }      
        //빨간불, 정지
        else
        {
            crosswalk_e_stop.data = 1;
            printf("    Red light\n");
            printf("     STOP!!!!\n");
        }     
    }
    else if(node_index >= stopline[3] - caution_deviation && node_index <= stopline[3]) //신호등, 직진
    {
        flag = check_traffic(0);//직진
        printf("Mission : Cross walk\n");
        //파란불
        if(flag)
            printf("    Green light\n");
        //빨간불, 감속    
        else if(stopline[3] - node_index > stop_deviation)
        {
            speed_reduction.data = 0.3;
            printf("    Red light\n");
        }      
        //빨간불, 정지
        else
        {
            crosswalk_e_stop.data = 1;
            printf("    Red light\n");
            printf("     STOP!!!!\n");
        }     
    }
    else                                            // normal course
        printf("Mission : Normal course\n");
    printf("\n---------------------------------------------------\n\n");
    */

    /*
    //k-city 본선
    
    int stopline[7] = {227, 280, 417, 524, 595, 892, 938};
    if(0)
    {

    }
    
    if(node_index >= 20 && node_index <= 81) //주차 구역
    {
        mission_num.data = parking_mission;
        speed_reduction.data = 0.5;
        if(park_seq == 6)
        {
            speed_reduction.data = 0.5;
        }
        printf("Mission : Parking area\n");
    }
    
    else if(node_index >= stopline[1] - caution_deviation && node_index <= stopline[1]) //신호등, 직진
    {
        flag = check_traffic(0);//직진
        printf("Mission : Cross walk\n");
        //파란불
        if(flag)
            printf("    Green light\n");
        //빨간불, 감속    
        else if(stopline[1] - node_index > stop_deviation)
        {
            speed_reduction.data = 0.3;
            printf("    Red light\n");
        }      
        //빨간불, 정지
        else
        {
            crosswalk_e_stop.data = 1;
            printf("    Red light\n");
            printf("     STOP!!!!\n");
        }     
    }
    else if(node_index >= 327 && node_index <= 399) //정적 장애물(대형장애물) 등장 구간
    {
        speed_reduction.data   = 0.5;
        available_path.data[0] = 0; //우측으로 차선변경 가능
        available_path.data[1] = 1;
        available_path.data[2] = 1; 
        printf("Mission : Static obstacle\n");
    }
    else if(node_index >= stopline[2] - caution_deviation && node_index <= stopline[2]) //신호등, 직진
    {
        flag = check_traffic(0);//직진
        printf("Mission : Cross walk\n");
        //파란불
        if(flag)
            printf("    Green light\n");
        //빨간불, 감속    
        else if(stopline[2] - node_index > stop_deviation)
        {
            speed_reduction.data = 0.3;
            printf("    Red light\n");
        }      
        //빨간불, 정지
        else
        {
            crosswalk_e_stop.data = 1;
            printf("    Red light\n");
            printf("     STOP!!!!\n");
        }     
    }
    else if(node_index >= stopline[3] - caution_deviation && node_index <= stopline[3]) //신호등, 좌회전
    {
        flag = check_traffic(0);//직좌회전
        printf("Mission : Cross walk\n");
        //파란불
        if(flag)
            printf("    Green light\n");
        //빨간불, 감속    
        else if(stopline[3] - node_index > stop_deviation)
        {
            speed_reduction.data = 0.3;
            printf("    Red light\n");
        }      
        //빨간불, 정지
        else
        {
            crosswalk_e_stop.data = 0;
            printf("    Red light\n");
            printf("     STOP!!!!\n");
        }     
    }
    else if(node_index >= stopline[5] - caution_deviation && node_index <= stopline[5]) //신호등, 직진
    {
        flag = check_traffic(0);//직진
        printf("Mission : Cross walk\n");
        //파란불
        if(flag)
            printf("    Green light\n");
        //빨간불, 감속    
        else if(stopline[5] - node_index > stop_deviation)
        {
            speed_reduction.data = 0.3;
            printf("    Red light\n");
        }      
        //빨간불, 정지
        else
        {
            crosswalk_e_stop.data = 1;
            printf("    Red light\n");
            printf("     STOP!!!!\n");
        }     
    }
    else if(node_index >= stopline[6] - caution_deviation && node_index <= stopline[6]) //신호등, 직진
    {
        flag = check_traffic(0);//직진
        printf("Mission : Cross walk\n");
        //파란불
        if(flag)
            printf("    Green light\n");
        //빨간불, 감속    
        else if(stopline[6] - node_index > stop_deviation)
        {
            speed_reduction.data = 0.3;
            printf("    Red light\n");
        }      
        //빨간불, 정지
        else
        {
            crosswalk_e_stop.data = 1;
            printf("    Red light\n");
            printf("     STOP!!!!\n");
        }     
    }
    else if(node_index >= 700 && node_index <= 778)
    {
        speed_reduction.data = 0.65;
    }
    else                                            // normal course
        printf("Mission : Normal course\n");
    printf("\n---------------------------------------------------\n\n");
    */
   
    //똥멍청이 판교
    if(node_index >= 88 && node_index <= 120)      //start line
    {
        //how to start? -> start at vel_planing
    }
    else if(node_index >= 140 && node_index <= 174) //target car
    {
        speed_reduction.data = 0.7;
        printf("Mission : Safe zone\n");
    }
    else if(node_index >= 180 && node_index <= 190) //parking
    {
        speed_reduction.data = 0.7;
        printf("Mission : Safe zone\n");
    }
    else if(node_index >= 191 && node_index <= 226) //safe zone
    {
        mission_num.data = dy_obs_mission;
        printf("Mission : Dynamic obstacle\n");
    }
    else                                            // normal course
        printf("Mission : Normal course\n");
    printf("\n---------------------------------------------------\n\n");
}


int main(int argc, char **argv)
{
    //Initialize the node and resister it to the ROSMASTER
    ros::init(argc, argv, "mission_identifiying");
    //Declare node handle
    ros::NodeHandle nh;
    //Declare sub,pub
    ros::Subscriber mission_location_sub  = nh.subscribe("index",1,baseCALLBACK);
    ros::Subscriber traffic_sign_identify = nh.subscribe("/darknet_ros/bounding_boxes",100,trafficCallBack);
    ros::Subscriber load_identify         = nh.subscribe("/lane_mission",1,laneCallBack);
    ros::Subscriber mia_parking_sub       = nh.subscribe("/parking",1, parkCallBack);

    ros::Publisher crosswalk_E_stop_pub   = nh.advertise<std_msgs::Int32>("/crosswalk_E_stop", 10);
    ros::Publisher available_path_pub     = nh.advertise<std_msgs::Int32MultiArray>("/available_path",10);
    ros::Publisher speed_reduction_pub    = nh.advertise<std_msgs::Float64>("/speed_reduction",10);
    ros::Publisher mission_num_pub        = nh.advertise<std_msgs::Int32>("/mission_num",10);
    

    ros::Rate loop_rate(50);
    while(ros::ok())
    {
        ros::spinOnce(); 
        mission_identify();
        bayes_traffic();
        crosswalk_E_stop_pub.publish(crosswalk_e_stop);
        mission_num_pub.publish(mission_num);
        speed_reduction_pub.publish(speed_reduction);
        available_path_pub.publish(available_path);
        loop_rate.sleep();
    }   
    return 0;
}
