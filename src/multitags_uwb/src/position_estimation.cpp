#include "ros/ros.h"
#include "stdio.h"
#include "stdlib.h"
#include "std_msgs/String.h"
#include "math.h"
#include <iostream>
#include <fstream>
#include <string>
#include <numeric>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include "position_estimation.h"

int num_tags = 3;
float GD_alpha = 1.0e-3;
Solution solution_inst;
Tag All_tags[num_tags];
Anchor The_anchor;
geometry_msgs::Pose Robot_Absolute_Pose;
geometry_msgs::Pose Anchor_Relative_Pose;

void callback_dists(std_msgs::String msg){

}

int main(int argc, char **argv){
	ros::init(argc,argv,"position_estimation");
	ros::NodeHandle n;
	ros::Rate loop_rate(100.0);
	ros::Subscriber sub_dists = n.subscribe("ALL_DIST",10,callback_dists);
	ros::Publisher pub_absolute_pos_robot = n.advertise<geometry_msgs::Pose>("Robot_Absolute_Pose",10);
	ros::Publisher pub_relative_pos_anchor = n.advertise<geometry_msgs::Pose>("Anchor_Relative_Pose",10);
	ros::Time time_stmp = ros::Time::now();
	solution_inst.Init(num_tags, The_anchor, All_tags, GD_alpha);

	while (ros::ok()){
		//-----------------------------------------------------------------------------------------------
		//(START) measuring the frequency of the localziation estimation process
		long double period = (ros::Time::now() - time_stmp).toSec();
		time_stmp = ros::Time::now();
		long double frequency = 1.0/period;
		//(END) measuring the frequency of the localziation estimation process
		//---------------------------------------------------------------------------------------
		//(START) main computations and publishing estimated positions
		solution_inst.Gradient_Descent_Solver();
		Robot_Absolute_Pose.position.x = solution_inst.robot.global_x;
		Robot_Absolute_Pose.position.y = solution_inst.robot.global_y;
		Robot_Absolute_Pose.position.z = solution_inst.robot.global_z;
		Anchor_Relative_Pose.position.x = solution_inst.anchor_inst.local_x;
		Anchor_Relative_Pose.position.y = solution_inst.anchor_inst.local_y;
		Anchor_Relative_Pose.position.z = solution_inst.anchor_inst.local_z;
		pub_absolute_pos_robot.publish(Robot_Absolute_Pose);
		pub_relative_pos_anchor.publish(Anchor_Relative_Pose);
		//(END) main computations and publishing estimated positions
		//---------------------------------------------------------------------------------------------
		ros::spinOnce();
		loop_rate.sleep();
	}
}

