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
#include "std_msgs/Float64.h"
#include "darknet_ros_msgs/BoundingBox.h"
#include "darknet_ros_msgs/BoundingBoxes.h"

//global
int node_index;
float stop_deviation=3;
float deviation,left_curv, right_curv;
int line_type,error,fair_state;
bool left_trun,right_trun;

bool flag=1;
float traffic_flag[10][4];

float read_right[3]={0.55,0.45,0.1};
float beyes_GRX[3]={0.333333,0.333333,0.333333};
float beyes_TL[2]={0.5,0.5};

enum traffic_information{
    GREEN,
    RED,
    CROSS,
    LEFT,
    RIGHT,
    CROSSWALK,
    SAFEZONE,
    HUMP,
    PARKING,
    TL
}traffic_information;

int GOSTRAIGHT=0;
int TURNLEFT=1;
int TURNRIGHT=2;

enum ahead_signal{
    NOTHING,
    DOTKNOW,
    STOPLINE,
    CHIN
}ahead_signal;

darknet_ros_msgs::BoundingBoxes traffic_signs;
std_msgs::Int32MultiArray available_path;
std_msgs::Float64 speed_reduction;

// double crosswalk_checker(enum crosswalker){
//     if(ahead_signal==STOPLINE){
//         if(stop_deviation<10){
//             if(crosswalker==GOSTRAIGHT){
//                 if(traffic_flag[0]>0.6) speed_reduction.data=1.0;
//                 else if(traffic_flag[1]>0.6) speed_reduction.data=0.0;
//                 else{speed_reduction.data=1.0;}
//             }
//             else if(crosswalker==TURNLEFT){
//                 if(traffic_flag[9]>0.6) speed_reduction.data=1.0;
//                 else{speed_reduction.data=0.0;}
//             }
//             else if(crosswalker==TURNRIGHT){
//                 speed_reduction.data=0.5;
//             }
//             else{
//                 printf("Are you crazy h.u.m.a.n?");
//             }
//         }
//     }

// }


void trafficCallBack(const darknet_ros_msgs::BoundingBoxes::ConstPtr& detection)
{

    for(int i = 0 ; i < detection->bounding_boxes.size(); i++){
        traffic_flag[detection->bounding_boxes[i].Class][3] = (-detection->bounding_boxes[i].ymin+detection->bounding_boxes[i].ymax)*(-detection->bounding_boxes[i].xmin+detection->bounding_boxes[i].xmax);

        if(traffic_flag[detection->bounding_boxes[i].Class][3]>200){
            traffic_flag[detection->bounding_boxes[i].Class][0] = detection->bounding_boxes[i].probability;
            traffic_flag[detection->bounding_boxes[i].Class][1] = (detection->bounding_boxes[i].xmin+detection->bounding_boxes[i].xmax)/2;
            traffic_flag[detection->bounding_boxes[i].Class][2] = (detection->bounding_boxes[i].ymin+detection->bounding_boxes[i].ymax)/2;
        }
    }

}

void baseCALLBACK(const std_msgs::Int32::ConstPtr& index)
{
    node_index = index->data;
}



void laneCallBack(const macaron::Floats_for_mission::ConstPtr& mission)
{
    stop_deviation=mission->stop_deviation;
    deviation=mission->deviation;
    left_curv=mission->left_curv;
    right_curv=mission->right_curv;
    line_type=mission->line_type;
    error=mission->error;
    fair_state=mission->fair_state;
    left_trun=mission->left_trun;
    right_trun=mission->right_trun;
    printf("hi");

}

double cross(int signal){
    double a;
    if(stop_deviation<10){
        if(signal==GOSTRAIGHT){
            if(beyes_GRX[2]>beyes_GRX[0] && beyes_GRX[2]>beyes_GRX[1])  a = 1.0;
            else if(beyes_GRX[0]>beyes_GRX[1])  a = 1.0;
            else if(beyes_GRX[0]<beyes_GRX[1] && stop_deviation<2) a = 0.0;
            else if(beyes_GRX[0]<beyes_GRX[1]) a = 0.3;
        }

        else if(signal==TURNLEFT){
            if(beyes_TL[1]>0.8)  a = 1.0;
            else if((beyes_TL[1]<0.2) && stop_deviation<4) a = 0.0;
            else if((beyes_TL[1]<0.2)) a = 0.3;
        }
        else if(signal==TURNRIGHT){
            a = 0.5;
        }
        else{
            a=1;
        }

    }
    else
    {
        a=1;
    }
    
    return a;
}

void bayes_traffic()
{
        double G,R,X,a;
        double turn_left,tx;

        if(traffic_flag[9][0]<0.2 && beyes_TL[0]<0.95){
            tx=beyes_TL[0]*0.9;
            turn_left=beyes_TL[1]*0.1;
            beyes_TL[0]=turn_left/(turn_left+tx);
            beyes_TL[1]=tx/(turn_left+tx);

        }
        else if(traffic_flag[9][0]>0.2 && beyes_TL[1]<0.95){
            tx=beyes_TL[0]*0.4;
            turn_left=beyes_TL[1]*0.6;
            beyes_TL[0]=turn_left/(turn_left+tx);
            beyes_TL[1]=tx/(turn_left+tx);
        }


        if(traffic_flag[0][0]<0.2 && traffic_flag[1][0]<0.2 && beyes_GRX[2]<0.95){
        G=beyes_GRX[0]*0.1;
        R=beyes_GRX[1]*0.1;
        X=beyes_GRX[2]*0.8;

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
    available_path.data.resize(3);
    available_path.data[0] = 0; //차선 변경 불가
    available_path.data[1] = 1;
    available_path.data[2] = 0;

    //팔정도 모의 실험
    /*
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
    else if(node_index >= 191 && node_index <= 226) //static obstacle
    {
        available_path.data[0] = 1; //좌측으로 차선변경 가능
        available_path.data[1] = 1;
        available_path.data[2] = 0;
        printf("Mission : Static obstacle\n");
    }
    else                                            // normal course
        printf("Mission : Normal course\n");
    printf("\n---------------------------------------------------\n\n");
    */
    //만해광장 장애물 회피 실험
    available_path.data[0] = 0; //오른쪽으로 차선변경
    available_path.data[1] = 1;
    available_path.data[2] = 1;
    //printf("Mission : Static obstacle\n");
    //printf("\n---------------------------------------------------\n\n");

    //k-city 예선
    /*
    if(node_index >= 124 && node_index <= 153)      //정적 장애물(드럼 통) 등장 구간
    {
        available_path.data[0] = 1; //좌측으로 차선변경 가능
        available_path.data[1] = 1;
        available_path.data[2] = 0; 
        printf("Mission : Static obstacle\n");
    }
    else if(node_index >= 216 && node_index <= 232) //신호등, 직진, 232가 정지선
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
    else if(node_index >= 271 && node_index <= 285) //신호등, 좌회전, 285가 정지선
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
    else if(node_index >= 316 && node_index <= 336) //돌방 장애물 등장 구간
    {
        printf("Mission : Dynamic obstacle\n");
    }
    else if(node_index >= 361 && node_index <= 380) //신호등, 좌회전, 380이 정지선
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
    else if(node_index >= 411 && node_index <= 427) //신호등, 직진, 427이 정지선
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
    else                                            // normal course
        printf("Mission : Normal course\n");
    printf("\n---------------------------------------------------\n\n");
    */

    //k-city 본선(아직 주차 제외)
    /*
    if(node_index >= 213 && node_index <= 229)      //신호등, 좌회전, 229가 정지선
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
    else if(node_index >= 267 && node_index <= 283) //신호등, 직진, 283이 정지선
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
    else if(node_index >= 311 && node_index <= 400) //정적 장애물(대형장애물) 등장 구간
    {
        available_path.data[0] = 0; //우측으로 차선변경 가능
        available_path.data[1] = 1;
        available_path.data[2] = 1; 
        printf("Mission : Static obstacle\n");
    }
    else if(node_index >= 403 && node_index <= 419) //신호등, 직진, 419가 정지선
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
    else if(node_index >= 885 && node_index <= 901) //신호등, 직진, 901이 정지선
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
    else if(node_index >= 931 && node_index <= 947) //신호등, 직진, 947이 정지선
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
    else                                            // normal course
        printf("Mission : Normal course\n");
    printf("\n---------------------------------------------------\n\n");
    */
}


int main(int argc, char **argv)
{
    //Initialize the node and resister it to the ROSMASTER
    ros::init(argc, argv, "mission_identifiying");
    //Declare node handle
    ros::NodeHandle nh;
    //Declare sub,pub
    ros::Rate loop_rate(20);
    ros::Subscriber mission_location_sub  = nh.subscribe("index",1,baseCALLBACK);
    ros::Subscriber traffic_sign_identify = nh.subscribe("/darknet_ros/bounding_boxes",100,trafficCallBack);
    ros::Subscriber load_identify         = nh.subscribe("/lane_identify",1,laneCallBack);

    ros::Publisher available_path_pub     = nh.advertise<std_msgs::Int32MultiArray>("/available_path",10);
    ros::Publisher speed_reduction_pub    = nh.advertise<std_msgs::Float64>("/speed_reduction",10);
    
    while(ros::ok())
    {
        ros::spinOnce(); 
        mission_identify();
        bayes_traffic();
        speed_reduction.data=cross(GOSTRAIGHT);
        speed_reduction_pub.publish(speed_reduction);
        available_path_pub.publish(available_path);
        loop_rate.sleep();
    }   
}
