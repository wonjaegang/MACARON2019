//ros basic header
#include "ros/ros.h"
//message header

#define PI acos(-1)
#include "sensor_msgs/Image.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

#include <iostream>
#include "math.h"
#include "macaron/Floats.h"
#include "macaron/spoon.h"
#include "macaron/erp42_state.h"
//math header&constant

//global variables ## : changeable variables
double wheel_base = 1.040, tread = 0.985, width = 1.160; //macaron property
double angle_now = 0;                              
double w_offset = 1;                               //
double w_safety = 1;                               //
double w_consistency = 1;                          //



double wheel_base = 1.040, tread = 0.985, width = 1.160; //macaron property
double obstacle[2][181] = { 0 };                   
double angle_now = 0;                              
double d_camera_p = 1;                             // 

const int num_node = 6;                                 //
double P[2][num_node] = {{0,1,1,2,2,3},{0,0,1,1,2,2}};                     //
double P_vec[2][num_node] = { 0 };                 //

double dt = 0.01;                                  //
int Nsample = 1 / dt;
const int path_leng = 501;      // 
double base_path[2][path_leng];            // 
double base_path_curve[3][path_leng];      //s,intergal s,K

int localized_node;                                //
double qi;                                         //
double qf;                                         //
double search_range = 1;

const int candidate_num = 7;                             // 
const int look_ahead_num = 100;					   // 
double real_path[2][look_ahead_num + 1] = { 0 };   // 
double w_offset = 1;                               //
double w_safety = 1;                               //
double w_consistency = 1;                          //
double Cs[3][2*candidate_num+1] = { 0 };            //경로의 길이와 장애물 유무를 저장.

double gx(int i,double sigma){

	double gausian_factor=exp(-0.01*double(i)*double(i)/2/sigma/sigma)/sqrt(2*PI*sigma);
	return gausian_factor;
}


void generate_basepath()
{
	//14번 논문을 이용하여 거리에 대한 매개변수화를 하고
	//15번 논문을 이용하여 거리를 구한 뒤,
	//16번 논문을 이용하여 좌표를 구했다고 하더라. 근데 이거 16번 논문에 다 나온 내용인데?
	double alpha=0.5;
	for (int xy = 0; xy < 2; xy++)
	{	for (int big_node = 1; (big_node<num_node-1); big_node++){
			P_vec[xy][big_node]=(1-alpha)*(P[xy][big_node+1]-P[xy][big_node-1]);
	}
	}

	for (int xy = 0; xy < 2; xy++)
	{
		for (int big_node = 0; big_node < num_node - 1; big_node++)
		{
			double a = +2 * P[xy][big_node] - 2 * P[xy][big_node + 1] + 1 * P_vec[xy][big_node] + 1 * P_vec[xy][big_node + 1];
			double b = -3 * P[xy][big_node] + 3 * P[xy][big_node + 1] - 2 * P_vec[xy][big_node] - 1 * P_vec[xy][big_node + 1];
			double c = P_vec[xy][big_node];
			double d = P[xy][big_node];
			double t = 0;
			for (int small_node = 0; small_node < Nsample; small_node++)
			{
				base_path[xy][small_node + Nsample * big_node] = a*t*t*t + b*t*t + c*t + d;
				t += dt;
			}
		}

		base_path[xy][path_leng - 1] = P[xy][num_node];
	}
	
	for (int big_node = 0; big_node < num_node - 1; big_node++)
		{
			for (int small_node = 0; small_node < Nsample; small_node++){
				base_path_curve[0][small_node + Nsample * big_node]=sqrt((base_path[0][small_node + Nsample * big_node+1]-base_path[0][small_node + Nsample * big_node])*(base_path[0][small_node + Nsample * big_node+1]-base_path[0][small_node + Nsample * big_node])+(base_path[1][small_node + Nsample * big_node+1]-base_path[1][small_node + Nsample * big_node])*(base_path[1][small_node + Nsample * big_node+1]-base_path[1][small_node + Nsample * big_node]));
				base_path_curve[1][small_node + Nsample * big_node+1]=base_path_curve[1][small_node + Nsample * big_node]+base_path_curve[0][small_node + Nsample * big_node];
				printf("%f %f \n",base_path_curve[0][small_node + Nsample * big_node],base_path_curve[1][small_node + Nsample * big_node]);
			}
		}

	
}

void localization_to_path()
{
	double x0 = 0.5;
	double y0 = 0;
	double d_nearest = sqrt((x0 - P[0][0]) * (x0 - P[0][0]) + (y0 - P[1][0]) * (y0 - P[1][0])); // big node ��� ���� ���
	int nearest_bignode = 0;
	for (int big_node = 1; big_node < num_node; big_node++)
	{
		double TEMP = sqrt((x0 - P[0][big_node]) * (x0 - P[0][big_node]) + (y0 - P[1][big_node]) * (y0 - P[1][big_node]));
		if (TEMP < d_nearest)
		{
			d_nearest = TEMP;
			nearest_bignode = big_node;
		}
	}
	nearest_bignode = nearest_bignode * Nsample;
	localized_node = nearest_bignode;
	for (int small_node = nearest_bignode - Nsample; small_node < nearest_bignode + Nsample; small_node++)
	{
		if (small_node >= 0 && small_node < path_leng - 1)
		{
			double TEMP = sqrt((x0 - base_path[0][small_node]) * (x0 - base_path[0][small_node]) + (y0 - base_path[1][small_node]) * (y0 - base_path[1][small_node]));
			if (TEMP < d_nearest)
			{
				d_nearest = TEMP;
				localized_node = small_node;
			}
		}
	}
	//printf("%d",localized_node);
	int direction;
	double vehicle_angle=atan2(y0 - base_path[1][localized_node], x0 - base_path[0][localized_node]) - atan2(base_path[1][localized_node +1] - base_path[1][localized_node], base_path[0][localized_node +1] - base_path[0][localized_node]);
	if (vehicle_angle > 0)
		direction = 1;
	else
		direction = -1;
	qi = direction * d_nearest;
}


void generate_candidate_path()
{
double path_length;
for (int num = -candidate_num; num <= candidate_num; num++)
	{
	qf = qi + search_range / double(candidate_num) * double(num);

	for (int xy = 0; xy < 2; xy++)
	{
		for (int k = 0; k < look_ahead_num; k++)
		{

			double theta = angle_now - atan2(base_path[1][localized_node + 1] - base_path[1][localized_node], base_path[0][localized_node + 1] - base_path[0][localized_node]);
			int look_ahead_node = localized_node + look_ahead_num;
			if (look_ahead_node > path_leng - 2) 

				look_ahead_node = path_leng - 2;
			double ds = look_ahead_num;
			double a1 = (ds * tan(theta) - 2 * (qf - qi)) / (ds * ds * ds);
			double a2 = (qf - qi) / (ds * ds);
			for (int node = localized_node; node <= look_ahead_node; node++)
			{
				int s = node - localized_node;
				double offset = (s - ds)*(s - ds) * (a1*s - a2) + qf; // offset ��
				double T = atan2(base_path[1, node + 1] - base_path[1, node], base_path[0, node + 1] - base_path[0, node]);
				real_path[0][s] = base_path[0][node] + offset * cos(T + PI/2);
				real_path[1][s] = base_path[1][node] + offset * sin(T + PI / 2);
			} 
		}
	}

	for (int k = 0; k < look_ahead_num + 1; k++)
	{
		Cs[0][1] = 1;
		if(num>0)	Cs[0][candidate_num+num] = 1;


		for (int i = 0; i < 181; i++)
		{
			double d_to_obstacle = sqrt(pow((real_path[0][k] - obstacle[0][i]), 2) + pow((real_path[1][k] - obstacle[1][i]), 2));
			path_length = sqrt(pow((real_path[0][k+1] - real_path[0][k]), 2) + pow((real_path[1][k+1] - real_path[1][k]), 2));
		}
	}
				Cs[1][candidate_num+num] = path_length;
	}

	for (int num = -candidate_num; num <= candidate_num; num++){
	for (int num2 = -candidate_num; num2 <= candidate_num; num2++)
	{
		Cs[2][candidate_num+num]+=Cs[0][candidate_num+num2]*gx(num-num2,0.1);
	}	//printf("%f %f %f %f \n",gx(num,0.1),Cs[2][candidate_num+num],Cs[0][candidate_num+num],Cs[1][candidate_num+num]);
	}

}


double path_cost()
{
	//offset cost
	double cost_offset = fabs(qf);
	//safty cost
	double cost_safety = Cs[2][4];
	
	double cost_consistency = 0;

	return w_offset*cost_offset + w_safety*cost_safety + w_consistency*cost_consistency;
}


int main(int argc ,char **argv)
{
		generate_basepath(); 
		localization_to_path(); 
		generate_candidate_path(); 

		//printf("%f",base_path_curve[1][501]);


		double lowest_cost = 1024;
		double qf_proper;
		for (int num = -candidate_num; num <= candidate_num; num++)
		{
			double TEMP = path_cost(); 
			if (lowest_cost > TEMP)
			{
				lowest_cost = TEMP;
				qf_proper = qf;
			}
		}		
		qf = qf_proper;  


	return 0;	
}


/*
*/


	
