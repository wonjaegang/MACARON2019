#include "ros/ros.h"
#include "math.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include "macaron/base_frame.h"
#include "macaron/erp42_write.h"
#include "macaron/erp42_read.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"
#include <sensor_msgs/Imu.h>

#define PI acos(-1)


double angle_limit(double a)
{
  if(a>2*PI)  { a-=2*PI;}
  if(a<0)  { a+=2*PI;}
  return a;
}

//----------------control property--------------------------//
double wheel_base = 1.040, tread = 0.985, width = 1.160; //macaron property
double ld = 7;                    //ld (m), 정수로 설정할 것
#define candidate_path_leng 10    //후보경로의 절점개수
int erp_speed, erp_steer;
double z_rate;

//-------------------input value--------------------------//
//from path planner
double selected_path[2][int(candidate_path_leng)];

//from gps_txt reader
double base_path[2][int(candidate_path_leng)]  = { 0 };
double path_leng = 7;
//from Localization node
double x_tm;
double y_tm;
double heading; //도북기준 heading, rad

//from mission identifier
double speed_reduction = 1.0;

//-------------------write value--------------------------//
bool write_E_stop;
int write_gear  = 0;
int write_speed = 0;
int write_brake = 1;
int write_steer = 0;
int remote_stop = 0;
std_msgs::Float64 lookahead;


void s_pathCallBack(const std_msgs::Float64MultiArray::ConstPtr& path)
{
    for(int xy = 0; xy < 2; xy++)
    {
        for(int i = 0; i < int(candidate_path_leng); i++)
        {
            selected_path[xy][i] = path->data[i + int(candidate_path_leng) * xy];
        }
    }
}


void headingCallBack(const geometry_msgs::Vector3::ConstPtr& location)
{
    x_tm     = location->x;
    y_tm     = location->y;
    heading  = location->z;
}


void pathCallBack(const macaron::base_frame::ConstPtr& path) 
{   
    printf("Subscring base path from GPS TXT reader\n");
    for(int i = 0; i < int(candidate_path_leng); i++)
    {
        base_path[0][i]  = path->s_a[i]; //xy 좌표 아님, 각도와 거리
        base_path[1][i]  = path->s_d[i];
    }
    path_leng = path->ld;
}


void EncoderCallback(const macaron::erp42_read::ConstPtr& msg)
{
    erp_speed=msg->read_speed;
    erp_steer=msg->read_steer;
}


void reductionCallBack(const std_msgs::Float64::ConstPtr& reduction) 
{
    speed_reduction = reduction->data;
}

void ImuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{ if(msg->angular_velocity.x==msg->angular_velocity.x && msg->linear_acceleration.x==msg->linear_acceleration.x&&
  msg->orientation.w==msg->orientation.w)
  {
    z_rate=msg->angular_velocity.z;
  }
}


void remoteCallBack(const geometry_msgs::Twist::ConstPtr& remoteinput)
{
    double inputspeed = remoteinput->linear.x;
    double inputsteer = remoteinput->angular.z;
    write_brake = 1;
    //write_gear = 1 - inputspeed / fabs(inputspeed);
    if(inputspeed < 0)
        write_gear = 2;
    else
        write_gear = 0;
    write_speed = int(fabs(inputspeed) * 100 + 0.5);
    write_steer = int(inputsteer * 2000 + 0.5) * -1;
    if(inputspeed  + inputsteer == 0)
    {
        write_speed = 0;
        write_brake = 200;
    }
}



int main(int argc, char **argv)
{
    ros::init(argc,argv,"vel_planning");
    ros::NodeHandle nh;
    ros::Rate loop_rate(200);

    ros::Publisher vel_planner = nh.advertise<macaron::erp42_write>("erp_write", 1);
    ros::Publisher lookahead_publisher = nh.advertise<std_msgs::Float64>("lookahead", 1);

    macaron::erp42_write erp42;
    
    /*
    ros::Subscriber enc_dat             = nh.subscribe("/erp_read",1,EncoderCallback);
    ros::Subscriber imu_data            = nh.subscribe("/imu/data",1,ImuCallback);
    ros::Subscriber seleted_path_sub    = nh.subscribe("/selected_path",1, s_pathCallBack);
    ros::Subscriber vel_localization    = nh.subscribe("/heading",1, headingCallBack);
    ros::Subscriber vel_baseframe_sub   = nh.subscribe("/base_frame",1, pathCallBack);
    ros::Subscriber speed_reduction_sub = nh.subscribe("/speed_reduction",1, reductionCallBack);
    */

    ros::Subscriber remote_sub          = nh.subscribe("/joy_teleop/cmd_vel",1,remoteCallBack);

    while(ros::ok)
    {
        ros::spinOnce();
        
        /*
        printf("selected path : ");
        for(int xy = 0; xy < 2; xy++)
        {
            for(int i = 0; i < int(candidate_path_leng); i++)
            {
                printf("%f, ", selected_path[xy][i]);
            }
            printf("\n");
        }
        printf("=======================\n");

        printf("base path vec & d: ");
        for(int xy = 0; xy < 2; xy++)
        {
            for(int i = 0; i < int(candidate_path_leng); i++)
            {
                printf("%f, ", base_path[xy][i]);
            }
            printf("\n");
        }
        printf("=======================\n");


        double ld_x  = selected_path[0][int(candidate_path_leng) - 1];
        double ld_y  = selected_path[1][int(candidate_path_leng) - 1];
        double alpha = atan2(ld_y - y_tm, ld_x - x_tm) - heading; //현재 방향과 ld포인트 사이의 각도 차이 , 오른손나사, rad
        double ld_for_steer = sqrt(pow(ld_x - x_tm, 2) + pow(ld_y - y_tm, 2));
        //steer control
        write_steer = 71 * atan2(2*wheel_base*sin(alpha), ld_for_steer / 2.0)*180/PI + 0.5;// + z_rate * PI/180; //conversion with rounding, ERP42는 시계방향 회전이 +이다
        if(write_steer > 2000)
            write_steer = 2000;
        else if(write_steer < -2000)
            write_steer = -2000;

        //velocity contrtol
        
        double kappa_max_TEMP=0;

        for(int i = 0; i < int(candidate_path_leng-2); i++)
        {
            double max_kappa = fabs(angle_limit(base_path[0][i+1]-base_path[0][i])/base_path[1][i+1]);
            printf("%f  ", max_kappa);
            if(max_kappa > kappa_max_TEMP)
            {
                kappa_max_TEMP = max_kappa;
            }
        }
        printf("=======================\n");

        //double kappa = cos(fabs(angle_limit(base_path[0][int(candidate_path_leng-1)] - heading)));  //거리로 나누지 않은 곡률값 (단순히 각도 차이)
        double kappa = fabs(2*sin(alpha)/ld_for_steer);  //거리로 나누지 않은 곡률값 (단순히 각도 차이)
        if (kappa>0.908) {kappa=0.908;}

         write_speed = 20 / ( kappa );
         if(write_speed > 100) { write_speed = 100;}   

         else if(write_speed<35) { write_speed = 35;}

        //write_speed = int((2000.0 - fabs(write_steer)) / 2000.0 * (60 - 30) + 0.5) + 30;
        
        ld = erp_speed/22.0 + 2;        
        if(ld > 15) {ld = 15;}   
         
        //ld = 3;

        lookahead.data = ld;
        erp42.write_speed = int(write_speed * speed_reduction + 0.5); //여기에 말고 최대속도에 곱하는 방안을 생각해보자.
        erp42.write_steer = write_steer;
        erp42.write_gear  = 0;
        erp42.write_brake = 1;
        lookahead_publisher.publish(lookahead);   
        vel_planner.publish(erp42);     
        */
        erp42.write_speed = write_speed;
        erp42.write_steer = write_steer;
        erp42.write_gear  = write_gear;
        erp42.write_brake = write_brake;
        vel_planner.publish(erp42);  
        printf("Speed : %d\n",write_speed);
        printf("Steer : %d\n",write_steer);
        printf("Gear  : %d\n",write_gear);
        printf("Brake : %d\n",write_brake);
        printf("=======================\n");


        //printf("%f %f\n",y_tm,kappa);
        /*
        printf("ld_x : %f  ld_y : %f\n", ld_x,ld_y);
        printf("base path_vec[0]     : %fdegree\n", base_path[0][int(candidate_path_leng-1)]*180/PI);
        printf("path_heading         : %fdegree\n", atan2(ld_y-y_tm,ld_x- x_tm) / PI * 180);
        printf("my heading           : %fdegree\n" ,heading / PI * 180);
        printf("Max Kappa            : %f\n", kappa_max_TEMP);
        printf("Selected alpha       : %fdegree\n",alpha / PI * 180);
        printf("Selected steer angle : %fdegree\n",write_steer / 71.0);
        printf("Selected velocity    : %d\n",erp42.write_speed);
        printf("lookahead length     : %f  %f \n",ld, kappa);
        printf("Speed reduction      : %f\n",speed_reduction);
        printf("---------------------------------------------------\n");
        */
        loop_rate.sleep();
    }
    return 0;
}
