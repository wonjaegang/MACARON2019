#include "ros/ros.h"

#include "sensor_msgs/Image.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Odometry.h"

#include <iostream>
#include "math.h"
#include "macaron/Floats.h"
#include "macaron/erp42_read.h"
#include "macaron/erp42_write.h"

#define PI acos(-1)

//-------------------input value--------------------------//
//from lanetraker
double chasun=0;
double deviation = 0;
double deviation_I = 0;


//from path_planner
double macaron_look_ahead=2;
double macaron_offset=0;
double macaron_orient=0;
//from erp_reader
double velocity,accel,s,l,yaw;
int AroM,brake,steer,speed,E_stop,gear,ENC,angle;
//from lidar
const int candidate_num=60;
double scan_data[2][180];
double carte_data[3][360];
double lane_data[2][23];
double cost[4][57];

//from RTK gps
double x_tm;
double y_tm;

/*
로컬에서 차량 각도구하기 : v*tan(kw)/W
로컬에서 좌우 이동거리 : v*sin(theta) 적분
로컬에서 전면 이동거리 : v*cos(tetha) 적분
*/

//-------------------write value--------------------------//
bool write_E_stop;
int write_gear=0;
int write_speed=0;
int write_brake=1;
int write_steer=0;

//----------------control property--------------------------//
double wheel_base = 1.040, tread = 0.985, width = 1.160; //macaron property
double w_offset = 1;                               //
double w_safety = 1;                               //
double w_consistency = 1;                          //

//1. lidar gausian fuction
//2. fab(x) from chasun center
//3. previous choice
//4. add all value

bool order_holder=0;

double gx(int i,double sigma){

    double gausian_factor=exp(-0.01*double(i)*double(i)/2/sigma/sigma)/sqrt(2*PI*sigma);
    return gausian_factor;
}


double radians(double deg)
{
    return deg / double(180) * PI; 
}


void laser_scan_Callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    for(int i=0; i<811; i++)
    {
        scan_data[0][i]=msg->ranges[i];
        scan_data[1][i]=0.3333*i-135;
        if(isinff(scan_data[0][i])) scan_data[0][i]=25;
    }
}


void erp42_reader(const macaron::erp42_read::ConstPtr& msg)
{
    AroM=msg->read_AorM;
    brake=msg->read_brake;
    steer=msg->read_steer;
    speed=msg->read_speed;
    E_stop=msg->read_E_stop;
    gear=msg->read_gear;
    ENC=msg->read_ENC;
    velocity=msg->read_velocity;
    accel=msg->read_accel;
    s=msg->read_s; //y
    l=msg->read_l; //x
    yaw=-(msg->read_yaw);//deg, from y axis. counterclockwise. by IMU sensor
}


void gpsCallBack(const sensor_msgs::NavSatFix::ConstPtr& fix) // lon, lat -> tm, gpstxt reader에서 받아도 될듯. 굳이 두번 계산?
{
    double lon =  fix->longitude;
    double lat =  fix->latitude;

    double e_2,e,C,T,A,N,M,M_0;    
    double st_lat = 38.0; // standard latitude
    double st_lon = 127.0; // standard longitude
    double k_0 =1.0;
    double a = 6378137.0;
    double b = 6356752.31;
    double f =(a-b)/a;
    double d_y = 200000.0;
    double d_x = 600000.0;

    e = (a*a - b*b) / (a*a);
    e_2 = (a*a - b*b) / (b*b);
    M = a * ((1 - e/4.0 - 3.0*e*e/64.0 - 5.0*pow(e,6.0)/256.0)*radians(lat) - (3.0*e/8.0 + 3.0*e*e/32.0 + 45.0*e*e*e/1024.0)*sin(2*radians(lat)) + (15.0*e*e/256.0 + 45.0*e*e*e/1024.0)*sin(4.0*radians(lat)) - 35.0*e*e*e/3072.0*sin(6.0*radians(lat)));
    C = (e / (1-e))*cos(radians(lat));
    T = pow(tan(radians(lat)),2);
    A = (radians(lon - st_lon))*cos(radians(lat));
    N = a/sqrt(1-(e)*(sin(radians(lat))*sin(radians(lat))));
    M_0 = a * ((1 - e/4.0 - 3.0*e*e/64.0 - 5.0*e*e*e/256.00)*radians(st_lat) - (3.0*e/8.0 + 3.0*e*e/32.0 + 45.0*e*e*e/1024.0)*sin(2.0*radians(st_lat)) + (15.0*e*e/256.0 + 45.0*e*e*e/1024.0)*sin(4.0*radians(st_lat)) - 35.0*e*e*e/3072.0*sin(6.0*radians(st_lat)));
    y_tm = (d_y+k_0*N*(A + (A*A*A/6.0)*(1-T+C) + (A*A*A*A*A/120.0) * (5.0 - 18.0*T + T*T + 72.0*C - 58.0*e_2)));
    x_tm = (d_x + k_0*(M - M_0 + N*tan(radians(lat))*(A*A/2.0 + (A*A*A*A/24.0)*(5.0-T+9.0*C+4.0*C*C)+(A*A*A*A*A*A/720.0)*(61.0-58.0*T+T*T+600.0*C-330.0*e_2))));
}


void lane_angle(const macaron::Floats::ConstPtr& lane)
{
    lane->mid_point_vector;
    lane->testset.data;
    lane->testset.layout;
    deviation=lane->deviation;
    deviation_I=deviation_I+0.1*deviation*AroM;
    float dstride0 = lane->testset.layout.dim[0].stride;
    float dstride1 = lane->testset.layout.dim[1].stride;
    float h = lane->testset.layout.dim[0].size;
    float w = lane->testset.layout.dim[1].size;
    double l_cost=1;
    for (int i=0; i<h; i++){
        for (int j=0; j<w; j++){
            lane_data[i][j]=lane->testset.data[dstride1*i+j];
          //  printf("%lf ",lane_data[i][j]);
    } //           printf("\n");
    }

    double a=0;
    for (int j=0; j<23; j++) 
        {
            a+=lane_data[1][j];
        }
    chasun=180/PI*atan(a/23);
    //printf("%f %f\n ",chasun,a);


    for(int num = -candidate_num; num <= candidate_num; num++){
        if(chasun>=num){l_cost=-0.1;}
        else{l_cost=0.1;}
            cost[1][candidate_num+num]=w_offset*l_cost*(num-chasun);
    }

    //캘리브레이션 코드가 절실하다 빠른시일내에 작성하도록
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"puppet");
    ros::NodeHandle nh;
    ros::Rate loop_rate(50);

    ros::Publisher control;
    control = nh.advertise<macaron::erp42_write>("erp_write", 1);
    macaron::erp42_write erp42;

    ros::Subscriber state_sub = nh.subscribe("erp_read",1, erp42_reader);
    ros::Subscriber laser_scan_data_received = nh.subscribe("scan",100, laser_scan_Callback);
    ros::Subscriber sun_sub = nh.subscribe("lane",10, lane_angle);

    //ros::Subscriber odome = nh.subscribe("/lanetraker",100, jaewon joa);

    while(ros::ok)
    {
        ros::spinOnce();


            double min=10000;

            for(int num = 0; num <= 2*candidate_num; num++)
            {
                if(cost[1][num]<min) 
                {
                    min=cost[1][num];
                    angle=(num-candidate_num);
                }
            }

        /*
        macaron_look_ahead=10;
        macaron_offset=-1;
        macaron_orient=0*PI/180;

        double a1=(macaron_look_ahead* tan(macaron_orient) - 2 * (macaron_offset - deviation)) / (macaron_look_ahead * macaron_look_ahead *macaron_look_ahead);
        double a2 = (macaron_offset - deviation)/ (macaron_look_ahead * macaron_look_ahead);


        double offset = (s - macaron_look_ahead)*(s - macaron_look_ahead) * (a1*s - a2) + macaron_offset; // offset   
        double T = 180/PI*atan((s- macaron_look_ahead)*(3*a1*s-2*a2-a1*macaron_look_ahead));
        if(s>macaron_look_ahead) T=0;

        double a=0;
        double K=2*sin(T*PI/180)/macaron_look_ahead;
        */



        //재원
    
        double x_offset = 5; // 원하는 tm 좌표
        double y_offset = 10;
        double theta_offset = 0 * PI / 180; //rad, from y axis
        double y = y_tm;
        double x = x_tm;
        double ld = 1;
        printf("x: %f, y: %f, yaw: %f\n",x, y, yaw);

        //x(y)  3차 함수 계수
        double b = (y_offset*y_offset*tan(theta_offset) - 3*x_offset*y_offset) / (y_offset*tan(theta_offset) - 2*x_offset);
        double a = x_offset / (y_offset*y_offset*y_offset + b*y_offset*y_offset);
        double T = atan(3*a*y*y - 2*a*b*y); // 접선벡터의 기울기(rad), 즉 atan(dx(y))

        
        write_steer=71 * atan2(2*wheel_base*sin(T - double(yaw)*PI/180), ld) * 180/PI;

        //-------------------------------경로 추종----------------------------------------------------------//
        //write_steer=71 * atan(2*wheel_base*sin(double(T-yaw)*PI/180)/macaron_look_ahead) * 180/PI;



        //-------------------------------차선 추종----------------------------------------------------------//

        //ver1. deviation과 chasun에 각각 가중치를 주는코드. k city 확인 완료. deviation에 강한가중치 가 필요.
        //write_steer=71 * atan(2*wheel_base*sin(double(chasun-10*deviation)*PI/180)/macaron_look_ahead) * 180/PI;

        //ver2. deviation에 의거해, chasun을 따라가는 코드, 실험필요
        //write_steer=71 * atan2(deviation + macaron_look_ahead * cos(PI/double(2) - chasun), macaron_look_ahead * sin(PI/double(2) - chasun)) * 180/PI;

        //ver3. deviation에 대한 PI제어
        //write_steer=71 * atan(2*wheel_base*sin(double(chasun-10*deviation-deviation_I)*PI/180)/macaron_look_ahead) * 180/PI;


        //ver4. GPS를 이용한 차선 추종(Potential Field)
        /* 
        int n = 2;//n번째 이후 노드(look_ahead_num)
        double n_x = 0, n_y = 0; //상대좌표로 n번째 이후 노드의 위치를 받음
        double heading = atan2(n_y, n_x);//나의 heading설정, grad
        double size_o_vec = 3;
        double alpha = 0.5;
        double lane_vec[2][22] = {0};
        lane_vec[0][0] = PI / double(2);
        for (int xy = 0; xy < 2; xy++)
        {	for (int node = 1; node < 22; node++)
            {
                lane_vec[xy][node] = (1-alpha) *(lane_data[xy][node+1]-lane_data[xy][node-1]);
            }
        }
        */

        //--------------------------------------------------------------------------------------------------//

        //printf("%d %f %f \n",write_steer,chasun*180/PI,deviation);
        if(write_steer>2000) write_steer=2000;
        else if(write_steer<-2000) write_steer=-2000;


        erp42.write_speed=30;

        if(y >= y_offset)
        {
            erp42.write_speed = 0;
            erp42.write_brake = 200;    
        }            


        erp42.write_steer=write_steer;
        erp42.write_gear=0;
        erp42.write_brake=1;
        //printf("%d %f %f\n",angle,desire_speed,desire_steer);

        //ROS_INFO("%f   %f   %f   %f",robot_oriention*180/angle_max, twist.linear.x, direction_r[3]*180/angle_max,direction_l[3]*180/angle_max);
        control.publish(erp42);          // Publishes 'msg' message      
        loop_rate.sleep();



        //printf("%d %f\n",angle,min);
        /*

        룩어헤드 포인트=차량의 속도*k + 차량의 최소탐지 거리

        차량의 속도=stablity/곡률(일단 stabilty는 고정)
        차량이 원하는 각도=k1*chasun+k2*deviation+미래각도...
        if gps값 이 있다면...
        input_v=
        스티어링=
        desire_
        */
        /*
        desire_speed=1*AroM;
        look_ahead_point=desire_speed+4;


        //desire_steer=71*atan(2*wheel_base*sin(double(chasun)*PI/180)/look_ahead_point)*180/PI;
        desire_steer=chasun*71;



        error=desire_speed-velocity;
        error_I=error_I+error*0.02*AroM;
        printf("%f %f l: %f s: %f, v: %f desire_steer: %f\n",error,error_I,l,s,v,chasun);

        erp42.brake=1;
        erp42.speed=75*error+10-accel*0.2;
        if(erp42.speed<0) erp42.wrtspeed=0;

        */
    }

    //ros::init(argc,argv,"spoon_sub");
    //ros::NodeHandle nh1;

    return 0;
}
