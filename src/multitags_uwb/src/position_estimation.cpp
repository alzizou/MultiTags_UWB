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
int current_tag_info;

void callback_dists(const std_msgs::String::ConstPtr& msg)
{
    auto jsonObject = json::parse(msg->data.c_str());
    for (int i=0;i<num_tags;i++){
	current_tag_info = jsonObject[i].get<int>();
	solution_inst.Tag[i].ID = current_tag_info[0];
	solution_inst.Tag[i].dist = current_tag_info[1];
	solution_inst.Tag[i].diff_dist = current_tag_info[2];
    }
}

void callback_attitude(const geometry_msgs::Quaternion& msg){
    data_orient = msg;

    long double sinr_cosp = (long double)(2.0 * (data_orient.w * data_orient.x + data_orient.y * data_orient.z));
    long double cosr_cosp = (long double)(1.0 - 2.0 * (data_orient.x * data_orient.x + data_orient.y * data_orient.y));
    solution_inst.robot_att.roll = (long double)(atan2(sinr_cosp, cosr_cosp));

    long double siny_cosp = (long double)(2.0 * (data_orient.w * data_orient.z + data_orient.x * data_orient.y));
    long double cosy_cosp = (long double)(1.0 - 2.0 * (data_orient.y * data_orient.y + data_orient.z * data_orient.z));
    solution_inst.robot_att.yaw = (long double)(atan2(siny_cosp, cosy_cosp));

    long double sinp = (long double)(2.0 * (data_orient.w * data_orient.y - data_orient.z * data_orient.x));
    if (abs_fcn(sinp) >= 1){
	solution_inst.robot_att.pitch = abs_fcn(sinp) * M_PI / 2.0;
    }else{
	solution_inst.robot_att.pitch = (long double)(asin(sinp));
    }

    solution_inst.robot_att.quat.w = data_orient.w;
    solution_inst.robot_att.quat.x = data_orient.x;
    solution_inst.robot_att.quat.y = data_orient.y;
    solution_inst.robot_att.quat.z = data_orient.z;
}

int main(int argc, char **argv){
	ros::init(argc,argv,"position_estimation");
	ros::NodeHandle n;
	ros::Rate loop_rate(100.0);
	ros::Subscriber sub_dists = n.subscribe("ALL_DIST",10,callback_dists);
	ros::Subscriber sub_att = n.subscribe("ROV_PX4_Orient",10, callback_attitude);
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
		solution_inst.Pos_Estimate();
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


long double abs_fcn(long double inp){
    long double out = inp;
    if (inp < 0.0){
	out = -1.0 * out;
    }
    return out;
}

long double sgn_fcn(long double inp){
    long double out = 0.0;
    if (inp < 0.0){
	out = -1.0;
    }
    if (inp > 0.0){
	out = +1.0;
    }
    return out;
}
