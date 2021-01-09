#include "ros/ros.h"
#include <iostream>
#include "math.h"

#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "macaron/Floats.h"
#include "macaron/erp42_write.h"


#define PI acos(-1)

//-------------------input value--------------------------//


//from gps_RTK
double x_tm;
double y_tm;
double vel;
double heading;

//from IMU
double imu_yaw; //도북기준 yaw, rad

//-------------------write value--------------------------//
double x,y,z;
double roll, pitch, yaw;

double radians(double deg)
{
    return deg / double(180) * PI; 
}


struct Quaternion
{
    double w, x, y, z;
};
struct EulerAngles
{
    double roll, pitch, yaw;
};

EulerAngles ToEulerAngles(Quaternion q)
{
    EulerAngles angles;
    // roll (x-axis rotation)
    double sinr_cosp = +2.0 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    angles.roll = atan2(sinr_cosp, cosr_cosp);
    // pitch (y-axis rotation)
    double sinp = +2.0 * (q.w * q.y - q.z * q.x);
    if (fabs(sinp) >= 1)
        angles.pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = asin(sinp);
    // yaw (z-axis rotation)
    double siny_cosp = +2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);  
    angles.yaw = atan2(siny_cosp, cosy_cosp);
    return angles;
}

void imuCallBack(const sensor_msgs::Imu::ConstPtr& msg) // 지금은 도북기준이 아니라 시작위치 기준이다. 도북기준으로 바꿀 것
{
    Quaternion q;
    q.x = msg->orientation.x;
    q.y = msg->orientation.y;
    q.z = msg->orientation.z;   
    q.w = msg->orientation.w;
    EulerAngles e = ToEulerAngles(q);
    yaw = e.yaw; //rad
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


void pathCallBack(const sensor_msgs::NavSatFix::ConstPtr& path) 
{
    localized_p_before[0] = localized_p[0];
    localized_p_before[1] = localized_p[1];
    localized_p[0]        = path->altitude;
    localized_p[1]        = path->altitude;
    localized_vec         = path->altitude;
    destination_p[0]      = path->altitude;
    destination_p[1]      = path->altitude;
    destination_vec       = path->altitude;
    ld_p[0]               = path->altitude;
    ld_p[1]               = path->altitude;
    ld_vec                = path->altitude;
    qi                    = path->altitude;
}


void generate_candidate_path(int index)
{
    qf_before = qf;
    qf = search_range / double(candidate_num) * double(index);
    double theta = yaw - destination_vec;
    double ds = candidate_path_leng;
    double a1 = (ds * tan(theta) - 2 * (qf - qi)) / (ds * ds * ds);
    double a2 = (qf - qi) / (ds * ds);
    

    int s = ld;
    double offset = (s - ds)*(s - ds) * (a1*s - a2) + qf;
    double x = ld_p[0] + offset * cos(ld_vec + PI/2);
    double y = ld_p[1] + offset * sin(ld_vec + PI/2);
    alpha = yaw - atan2(y_tm - y, x_tm - x); //시계반대가+ (오른손나사), rad
}


double path_cost()
{
    //offset cost
	double cost_offset = fabs(qf);

	//safty cost
    double cost_safety = 0;

	//consistency cost
	double common_s = candidate_path_leng - sqrt(pow(localized_p[0] - localized_p_before[0], 2) + pow(localized_p[1] - localized_p_before[1], 2));
	double cost_consistency = fabs(qf - qf_before) * common_s * 0.5;

	return w_offset*cost_offset + w_safety*cost_safety + w_consistency*cost_consistency;
}


void toERP42()
{
    write_steer = 71 * atan2(2*wheel_base*sin(-alpha), ld)*180/PI + 0.5; //conversion with rounding
    if(write_steer > 2000)
        write_steer = 2000;
    else if(write_steer < -2000)
        write_steer = -2000;

    erp42.write_speed = 30;
    erp42.write_steer = write_steer;
    erp42.write_gear  = 0;
    erp42.write_brake = 1;
}


int main(int argc, char **argv)
{
    ros::init(argc,argv,"path_planning");
    ros::NodeHandle nh;
    ros::Rate loop_rate(50);

    ros::Publisher control = nh.advertise<macaron::erp42_write>("/erp_write", 1);
    ros::Subscriber lidar_sub   = nh.subscribe("/scan",100, laser_scan_Callback);
    ros::Subscriber lane_sub    = nh.subscribe("/lane",10, laneCallBack);
    ros::Subscriber pp_gps_sub  = nh.subscribe("/fix",10, gpsCallBack);
    ros::Subscriber pp_imu_sub  = nh.subscribe("/imu_rpy",1,imuCallBack);
    ros::Subscriber gpspath_sub = nh.subscribe("/path",10, pathCallBack);

    while(ros::ok)
    {
        ros::spinOnce();
        
        int proper_index;
        double TEMP = 100; //임의의 큰 수
        for(int index = -candidate_num; index < candidate_num; index++)
        {
            generate_candidate_path(index);
            if(path_cost() < TEMP)
            {
                TEMP = path_cost();
                proper_index = index;
            }
        }
        generate_candidate_path(proper_index);
        toERP42();
        control.publish(erp42);     
        loop_rate.sleep();
    }
    return 0;
}
