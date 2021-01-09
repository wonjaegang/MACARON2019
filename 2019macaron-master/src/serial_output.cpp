#include "ros/ros.h"
#include "serial/serial.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "math.h"
#include <string>

#define PI acos(-1)

//sudo chmod 766 /dev/ttyUSBn : give a right to execute, write, read
serial::Serial ser;
double t_desire = 3;//sec, integer, you should change array size also
uint8_t gear;
uint8_t speed1[50 * 3];
uint8_t steer1[50 * 3];
uint8_t steer0[50 * 3];
uint8_t steer1_save = 0x00;
uint8_t steer0_save = 0x00; 
int steer_save = 0;
int inputbool = 0;


int speed_planning(double t)
{
    double x_desire = 2;//m
    double vt = -6 * x_desire/(t_desire * t_desire * t_desire) * t *(t - t_desire); //mps
    return int((vt * double(3600) / double(1000)) * 10 + 0.5); // convert to kph
}


int steer_planning(double t, int direction)
{
    double q_desire = 10;//degree
    double q_at_t0 =  double(steer_save) / 71;
    double qt = (-q_desire / (t_desire * t_desire * t_desire) * t * t * (2 * t - 3 * t_desire)) * direction + q_at_t0;
    int steer_value = int(qt * 71 + 0.5);
    if(steer_value > 2000)
        return 2000;
    else if (steer_value < -2000)
        return -2000;
    else
        return steer_value;    
}


void wasdCALLBACK(const std_msgs::String::ConstPtr& msg)
{   
    inputbool = 1;
    if(msg->data == "w")
    {
        gear = 0x00;
        for(int k = 0; k < 50 * t_desire; k++)
        {
            double t = double(k) / double(50);
            speed1[k] = u_int8_t(speed_planning(t));
            steer1[k] = steer1_save;
            steer0[k] = steer0_save;
        }    
    }
    else if (msg->data == "s")
    {
        gear = 0x02;
        for(int k = 0; k < 50 * t_desire; k++)
        {
            double t = double(k) / double(50);//sec
            speed1[k] = u_int8_t(speed_planning(t));
            steer1[k] = steer1_save;
            steer0[k] = steer0_save;
        }
    }
    else if (msg->data == "a")
    {
        gear = 0x01;
        for(int k = 0; k < 50 * t_desire; k++)
        {
            double t = double(k) / double(50);//sec
            speed1[k] = 0x00;
            steer1[k] = uint8_t(steer_planning(t,-1) % 256);
            steer0[k] = uint8_t((steer_planning(t,-1) - int(steer1[k])) / 256);
        }
        steer0_save = steer0[50 * int(t_desire) - 1];
        steer1_save = steer1[50 * int(t_desire) - 1];
        steer_save = int(steer1_save + steer0_save * 0xFF);
    }
    else if (msg->data == "d")
    {
        gear = 0x01;
        for(int k = 0; k < 50 * t_desire; k++)
        {
            double t = double(k) / double(50);//sec
            speed1[k] = 0x00;
            steer1[k] = uint8_t(steer_planning(t,1) % 256);
            steer0[k] = uint8_t((steer_planning(t,1) - int(steer1[k])) / 256);
        }
        steer0_save = steer0[50 * int(t_desire) - 1];
        steer1_save = steer1[50 * int(t_desire) - 1];
        steer_save = int(steer1_save + steer0_save * 0xFF);
    }
    else
        printf("Input Error\n");
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "serial_output");
    ros::NodeHandle nh;
    ros::Subscriber serial_output_sub = nh.subscribe("/cmd",1000,wasdCALLBACK);
    try
    {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port");
        return -1;
    }
    if(ser.isOpen())
    {
        ROS_INFO_STREAM("Serial Port initialized");
    }
    else
        return -1;
    ros::Rate loop_rate(50);
    while(ros::ok())
    {
        uint8_t a;
        ros::spinOnce();
        if(inputbool)
        {
            for(int k = 0; k < 50 * t_desire; k++)
            {
                a++;
                double t = double(k) / double(50);
                uint8_t UPPER_to_PCU_Packet[14]={0x53,0x54,0x58,0x01,0x00,gear,0x00,speed1[k], steer0[k], steer1[k] ,0x01,a,0x0D,0x0A};
                //                                S    T    X   A/M Estop gear   speed                  steer        brake  ETX0 ETX1 
                ser.write(UPPER_to_PCU_Packet,14);
                printf("speed : %d\n",int(speed1[k]));
                printf("steer : %d\n",int(steer1[k] + steer0[k] * 0xFF));
                loop_rate.sleep();
            }            
            inputbool = 0;
        }
        else
        {
            a++;
            uint8_t UPPER_to_PCU_Packet[14]={0x53,0x54,0x58,0x01,0x00,0x01,0x00,0x00,steer0_save,steer1_save ,0x80,a,0x0D,0x0A};
            //                                S    T    X   A/M Estop gear   speed             steer          brake  ETX0 ETX1 
            ser.write(UPPER_to_PCU_Packet,14);
            loop_rate.sleep();
        }
        
    }
    return 0;
}
