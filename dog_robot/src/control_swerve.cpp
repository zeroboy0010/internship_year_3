#include <ros/ros.h>
#include <robot/velocity.h>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <std_msgs/Float64.h>

robot::velocity velocity;

float vx = 0;
float vy = 0;
float vth = 0;
float L = 0.6;
float W = 0.6;


ros::Publisher b1_m1, b2_m1, f1_m1, f2_m1, b1_t1, b2_t1, f1_t1, f2_t1;

void cmd_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
	float speed1 = 0;
          float speed2 = 0;
	float speed3 = 0;
	float speed4 = 0;
	float angle1 = 0;
	float angle2 = 0;
	float angle3 = 0;
	float angle4 = 0;
 	float A,B,C,D;
	velocity.vx = msg->linear.x;
	velocity.vy = msg->linear.y;
	velocity.vth = msg->angular.z;
	vx = velocity.vx;
	vy = velocity.vy;
	vth = velocity.vth;


	A = vx - vth*L/2;
	B = vx + vth*L/2;
	C = vy - vth*W/2;
	D = vy + vth*W/2;
	
	speed1 = sqrt((B*B) + (C*C));
	speed2 = sqrt((B*B) + (D*D));
	speed3 = sqrt((A*A) + (D*D));
	speed4 = sqrt((A*A) + (C*C));
	
	angle1 = atan2(B,C);
	angle2 = atan2(B,D);
	angle3 = atan2(A,D);
	angle4 = atan2(A,C);
	
	ROS_INFO("vx = %0.2f vy = %0.2f vth = %0.2f",vx,vy,vth);
	ROS_INFO("speed1 = %0.2f, speed2 = %0.2f, speed3 = %0.2f, speed4 = %0.2f",speed1,speed2,speed3,speed4);
	ROS_INFO("angle1 = %0.2f, angle2 = %0.2f, angle3 = %0.2f, angle4 = %0.2f", angle1,angle2, angle3, angle4);
	std_msgs::Float64 s1;
	std_msgs::Float64 s2;
	std_msgs::Float64 s3;
	std_msgs::Float64 s4;
	
	std_msgs::Float64 a1;
	std_msgs::Float64 a2;
	std_msgs::Float64 a3;
	std_msgs::Float64 a4;
	
	s1.data = speed1;
	s2.data = speed2;
	s3.data = speed3;
	s4.data = speed4;
	
	a1.data = angle1;
	a2.data = angle2;
	a3.data = angle3;
	a4.data = angle4;
	
	b1_m1.publish(a1);
	b2_m1.publish(a2);
	f1_m1.publish(a3);
	f2_m1.publish(a4);
		
	b1_t1.publish(s1);
	b2_t1.publish(s2);
	f1_t1.publish(s3);
	f2_t1.publish(s4);
	
}

int main(int argc, char** argv)
{
	ros::init(argc,argv, "control_swerve");
	ros::NodeHandle n;
	ros::Subscriber cmd_sub = n.subscribe("cmd_vel",10,cmd_callback);
	
	b1_m1 = n.advertise<std_msgs::Float64>("/swerve/b1_t1_controller/command",1000);
	b2_m1 = n.advertise<std_msgs::Float64>("/swerve/b2_t1_controller/command",1000);
	f1_m1 = n.advertise<std_msgs::Float64>("/swerve/f1_t1_controller/command",1000);
	f2_m1 = n.advertise<std_msgs::Float64>("/swerve/f2_t1_controller/command",1000);
	
	b1_t1 = n.advertise<std_msgs::Float64>("/swerve/b1_t2_controller/command",1000);
	b2_t1 = n.advertise<std_msgs::Float64>("/swerve/b2_t2_controller/command",1000);
	f1_t1 = n.advertise<std_msgs::Float64>("/swerve/f1_t2_controller/command",1000);
	f2_t1 = n.advertise<std_msgs::Float64>("/swerve/f2_t2_controller/command",1000);
	
	
	
	
	
	while (ros::ok())
	{	
		
		ros::spinOnce();
		
	}
	
	return 0;
}

