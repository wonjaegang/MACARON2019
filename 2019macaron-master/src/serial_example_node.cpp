
#include <ros/ros.h>
#include <serial/serial.h>
#include <macaron/erp42_read.h>
#include <macaron/erp42_write.h>
#include <std_msgs/Int32.h>
#include <parking_algorithm/Message1.h>
#define PI acos(-1)
#define MAX 18

serial::Serial ser;

//imu value
double yaw_imu = 0, yaw0_imu = 0;
int firstrun = 1;

//write value
int dynamic_E_stop;
int crosswalk_E_stop;
int park_E_stop;
int steer=0;
uint8_t E_stop,steer1,steer2,speed,brake,gear;

double wheel_base = 1.040, tread = 0.985, width = 1.160; //macaron property
double ENC_saver[4];
double vel_saver[4];
double dt=0;
int park_seq;
int count = 0;

void parkCallBack(const parking_algorithm::Message1::ConstPtr& park)
{
    park_seq = park->seq_num;
}


void write_callback(const macaron::erp42_write::ConstPtr& write)
{
    if(park_seq == 1)
    {
        if(count < 50)
            park_E_stop = 1;
        count++;
        park_E_stop = 0;
    }
    if(park_seq == 3)
    {
        gear = 0;
        steer= 1072;
        speed= 30;
        park_E_stop = 0;

    }
    else if(park_seq == 4)
    {
        park_E_stop = 1;
        printf("estop_on");
    }
    else if(park_seq == 5)
    {
        gear = 2;
        steer= 1072;
        speed= 70;
        park_E_stop = 0;

    }
    else
    {
        gear=write->write_gear;
        steer= write->write_steer;
        speed= write->write_speed;
        park_E_stop = 0;

    }
    steer1=(steer/256);
    steer2=(steer%256);
    if(steer<0) steer1=steer1-1;
    brake= write->write_brake;
}


void E_stop_CallBack(const std_msgs::Int32::ConstPtr& stop)
{   
    dynamic_E_stop =  stop->data;
}


void cross_E_stop_CallBack(const std_msgs::Int32::ConstPtr& stop)
{   
    crosswalk_E_stop = stop->data;
}


int main (int argc, char** argv){
    ros::init(argc, argv, "serial_example_node");
    ros::NodeHandle nh;

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();
    ros::Subscriber parking_sub    = nh.subscribe("/parking",1, parkCallBack);
    ros::Subscriber write_sub      = nh.subscribe("erp_write", 1, write_callback);
    ros::Subscriber E_stop_sub     = nh.subscribe("/E_stop", 1, E_stop_CallBack);
    ros::Subscriber crosswalk_stop = nh.subscribe("/crosswalk_E_stop", 1, cross_E_stop_CallBack);
    ros::Publisher read_pub = nh.advertise<macaron::erp42_read>("erp_read", 1);
    macaron::erp42_read erp42_state;
    try
    {
        ser.setPort("/dev/erp42");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
        }

    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");

    }else{
        return -1;
    }

    ros::Rate loop_rate(50);
        uint8_t answer[17]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
        uint8_t answer_quere[MAX]={0,};
        uint8_t answer_tester[1]={0x00};

     if(ser.available()){
           // ROS_INFO_STREAM("Reading from serial port");

            while(answer_tester[0]!=0x53){
                ser.read(answer_tester,1); 
                //printf("%x ",answer_tester[0]);
            }
            answer_tester[0]={0x00};
            ser.read(answer,17);

            if(answer[0]==0x54 && answer[1]==0x58)     
            {
                if(answer[15]==0x0D && answer[16]==0x0A)
            {   for(int i=0;i<17;i++)         printf("%x ",answer[i]);
                printf("\n");
                    erp42_state.read_AorM=bool(answer[2]);
                    erp42_state.read_E_stop=bool(answer[3]);
                    erp42_state.read_gear=answer[4];
                    erp42_state.read_speed=int(answer[5]);
                    erp42_state.read_brake=int(answer[9]);
                    erp42_state.read_ENC=int(answer[13])*256*256*256+int(answer[12])*256*256+int(answer[11])*256+int(answer[10]);
                    ENC_saver[0]=erp42_state.read_ENC;
                    ENC_saver[1]=erp42_state.read_ENC;
                    ENC_saver[2]=erp42_state.read_ENC;
                    ENC_saver[3]=erp42_state.read_ENC;

            }
        }
     }


    while(ros::ok()){
    ros::spinOnce();
    last_time = current_time;
    current_time = ros::Time::now();
    dt = (current_time - last_time).toSec();

    if(dynamic_E_stop || crosswalk_E_stop || park_E_stop)
    {
        E_stop=0x01;
        
    }
    else
    {
        E_stop=0x00;    
    }

    uint8_t a;
    a++;

//    printf("Input speed  : %d\n", speed);
//    printf("Input steer  : %d\n", steer);
//    printf("Input gear   : %d\n", gear);
//    printf("Input E-stop : %d\n", E_stop);
//    printf("========================\n");

    uint8_t ask[14]={0x53,0x54,0x58,0x01,E_stop,gear,0x00,speed,steer1,steer2,brake,a,0x0D,0x0A};
    ser.write(ask,14);//S  0x53
            for(int i=0; i<14;i++){    
                printf("%x ",ask[i]);
            }
            printf("%f \n",dt);
        

        if(ser.available()){
            ser.read(answer_quere,18);
            
            for(int i=0; i<MAX;i++){    
                printf("%x ",answer_quere[i]);
            }
            printf("%f \n",dt);
        }

    if(answer_quere[0]!=0x53 || answer_quere[1]!=0x54 || answer_quere[2]!=0x58 || answer_quere[16]!=0x0D || answer_quere[17]!=0x0A)
    {
        for(int i=0; i<MAX;i++){
            answer_quere[i]={0x00};
        }
        //while(answer_tester[0]!=0x0A){
        //    ser.read(answer_tester,1);
        //    ROS_INFO("DORMAMU %x",answer_tester[0]);
        //}
        ser.flushInput();

    }

    else{
        erp42_state.read_AorM=bool(answer_quere[3]);
        erp42_state.read_E_stop=bool(answer_quere[4]);
        erp42_state.read_gear=answer_quere[5];
        erp42_state.read_speed=int(answer_quere[6]);
        erp42_state.read_brake=int(answer_quere[10]);
        erp42_state.read_ENC=int(answer_quere[14])*256*256*256+int(answer_quere[13])*256*256+int(answer_quere[12])*256+int(answer_quere[11]);
    
        for(int i=2; i>=0; i--)
        {
            ENC_saver[i+1]=ENC_saver[i];
        }
        ENC_saver[0]=erp42_state.read_ENC;
        erp42_state.read_velocity=0.01651*((ENC_saver[0]-ENC_saver[2]))/2/dt;
    
        for(int i=2; i>=0; i--)
        {
        vel_saver[i+1]=vel_saver[i];
        }
        vel_saver[0]=erp42_state.read_velocity;

        erp42_state.read_accel=((vel_saver[0]-vel_saver[1]))/dt;
        erp42_state.read_steer=int(answer_quere[9])*256+int(answer_quere[8]);

        if(erp42_state.read_steer>32768) erp42_state.read_steer=erp42_state.read_steer-65536+1;
 
        erp42_state.read_yaw=0.01664*(ENC_saver[0]-ENC_saver[1])*tan(double(erp42_state.read_steer)/71*PI/180)/wheel_base;
        erp42_state.read_s=erp42_state.read_s+0.01652*(ENC_saver[0]-ENC_saver[1])*cos( erp42_state.read_yaw/180/PI); //사실 y임
        erp42_state.read_l=erp42_state.read_s+0.01652*(ENC_saver[0]-ENC_saver[1])*sin( erp42_state.read_yaw/180/PI); //사실 x임
        read_pub.publish(erp42_state);
        ser.flushInput();
        loop_rate.sleep();

    }
                  //엔코더로 yaw를 구하는 식. 보정필요, IMU로 대체.
                    //erp42_state.read_yaw=erp42_state.read_yaw+0.01651*(ENC_saver[0]-ENC_saver[1])*tan(double(erp42_state.read_steer)/71*PI/180)/wheel_base*180/PI;
 }
}            
        
