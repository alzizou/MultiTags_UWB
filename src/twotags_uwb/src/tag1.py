#!/usr/bin/env python3
import rospy
import serial
import socket
import json
import numpy as np
import math as mt
from geometry_msgs.msg import Quaternion, Vector3
from termcolor import colored
from distance import distance
from ekf_dist import EKF_DIST

serial_inst = serial.Serial("/dev/ttyS0")
serial_inst.baudrate = 115200

This_tag = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
This_tag.bind(('192.168.3.184',8100))
#This_tag.bind(('10.11.1.17',8100))
This_tag.settimeout(0.01)

Other_tag = ('192.168.3.226',8100)
#Other_tag = ('10.11.1.21', 8100)

Results_File_TwoTags_Dists = open(r"/home/ubuntu/ali_ws/TwoTags_UWB/logs/Results_File_TwoTags_Dists.txt","w+")
Results_File_TwoTags_Pose = open(r"/home/ubuntu/ali_ws/TwoTags_UWB/logs/Results_File_TwoTags_Pose.txt","w+")

anchor_str = ['C','E']
temp_anch_str = None
MEAS = {0,0}
DATA_Store = np.zeros([2,2])
DIST_tag1 = 0
DIST_tag1_flt = 0.0
DIST_tag2_flt = 0.0

class Anchor:
    x = 0.0
    y = 0.0
    z = 0.0

anchor_pos = Anchor()
anchor_pos.x = 0.0
anchor_pos.y = 0.0
anchor_pos.z = 100.0

Rover_Z_pos = 100.0
Rel_dist_tags = 600.0
rov_yaw = 0.0
rov_pose = Vector3() #mm
rov_pose.z = Rover_Z_pos
pose_x = 0.0
pose_y = 0.0

x_hat_ekf = np.zeros([3,1])
P_ekf = np.eye(3)

Xpos_hat_ekf = np.zeros([3,1])
Xpos_P_ekf = np.eye(3)

Ypos_hat_ekf = np.zeros([3,1])
Ypos_P_ekf = np.eye(3)

def clean_shutdown():
	serial_inst.close()
	Results_File_TwoTags_Pose.close()
	Results_File_TwoTags_Dists.close()

def callback_quat(msg):
	global rov_yaw
	siny_cosp = (2.0 * (msg.w * msg.z + msg.x * msg.y))
	cosy_cosp = (1.0 - 2.0 * (msg.y * msg.y + msg.z * msg.z))
	rov_yaw_rad = mt.atan2(siny_cosp, cosy_cosp)
	rov_yaw

def tag1():

	global anchor_str, temp_anch_str, DATA, DIST_tag1, jsn_data_tag2
	global anchor_pos, Rover_Z_pos, Rel_dist_tags, rov_pose, rov_yaw
	global x_hat_ekf, P_ekf, DIST_tag1_flt, DIST_tag2_flt
	global Xpos_hat_ekf, Xpos_P_ekf, pose_x
	global Ypos_hat_ekf, Ypos_P_ekf, pose_y


	time_stmp = 0.0
	rospy.init_node("tag1",anonymous=True)
	pub_RelPose = rospy.Publisher("ROV_Rel_Pos", Vector3, queue_size=1)
	rate = rospy.Rate(100)
	rospy.on_shutdown(clean_shutdown)
	serial_inst.close()

	rospy.Subscriber("ROV_Quat", Quaternion, callback_quat)

	while not rospy.is_shutdown():
		rate.sleep()
		period = float(rospy.get_time()) - time_stmp
		time_stmp = float(rospy.get_time())
		freq = 1.0/period
		print("Frequency of tag1 node: %.2f" % freq)
		#----------------------------------------------------------------------------------
		#(START) measuring distance of tag1 to the anchor
		serial_inst.open()
		serial_inst.flush()
		while (temp_anch_str==None):
			serial_inst.write(anchor_str[0].encode())
			temp_anch_str = serial_inst.readline()
		if (temp_anch_str!=None):
			MEAS = distance(serial_inst)
			if (MEAS[1] > 1):
				DATA_Store[0][0] = MEAS[0]
				DATA_Store[0][1] = MEAS[1]
			if (MEAS[1] <= 1):
				MEAS[0] = DATA_Store[0][0]
				MEAS[1] = DATA_Store[0][1]
			DIST_tag1 = MEAS[1]
			temp_anch_str = None
		print(colored(("Tag1 to Anchor Distance mm: %.6f %d \r\n" %
			(rospy.get_time(),DIST_tag1)),"green"))
		serial_inst.close()
		#(END) measuring distance of tag1 to the anchor
		#----------------------------------------------------------------------------------
		#(START) filtering the measured distance
		x_hat_ekf,P_ekf = EKF_DIST(DIST_tag1,x_hat_ekf,P_ekf,period)
		DIST_tag1_flt = x_hat_ekf[0][0]
		#(END) filtering the measured distance
		#----------------------------------------------------------------------------------
		#(START) receiving distance of tag2 to the anchor
		try:
			This_tag.sendto('1'.encode(),Other_tag)
			Resp, Addrs = This_tag.recvfrom(1000)
			jsn_data_tag2 = json.loads(Resp.decode())
			DIST_tag2_flt = int(jsn_data_tag2.get("dist"))
			print(colored(("Tag2 to Anchor Distance mm: %.6f %d \r\n" %
			        (rospy.get_time(),DIST_tag2_flt)),"green"))
		except socket.timeout:
			print("No ditance received from Tag2!")
		#(END) receiving distance of tag2 to the anchor
		#----------------------------------------------------------------------------------
		#(START) computing the 2D position of rover
		projected_dist_tag1 = DIST_tag1_flt * mt.cos(anchor_pos.z - Rover_Z_pos)
		projected_dist_tag2 = DIST_tag2_flt * mt.cos(anchor_pos.z - Rover_Z_pos)
		longer_dist = projected_dist_tag1
		shorter_dist = projected_dist_tag2
		if (projected_dist_tag2 > longer_dist):
			longer_dist = projected_dist_tag2
			shorter_dist = projected_dist_tag1
		param_l0 = ( mt.pow(Rel_dist_tags,2) + mt.pow(longer_dist,2) - mt.pow(shorter_dist,2) ) / (2.0 * longer_dist)
		param_r0 = mt.sqrt( abs(mt.pow(Rel_dist_tags,2) - mt.pow(param_l0,2)) )
		param_theta = mt.atan2(param_r0 , param_l0)
		param_dist_avg = 0.5 * (longer_dist + shorter_dist)
		pose_x = param_dist_avg * mt.cos(param_theta + rov_yaw)
		pose_y = param_dist_avg * mt.sin(param_theta + rov_yaw)
		#(END) computing the 2D position of rover
		#----------------------------------------------------------------------------------
		#(START) filtering the estimated positions
		Xpos_hat_ekf,Xpos_P_ekf = EKF_DIST(pose_x,Xpos_hat_ekf,Xpos_P_ekf,period)
		rov_pose.x = Xpos_hat_ekf[0][0]
		Ypos_hat_ekf,Ypos_P_ekf = EKF_DIST(pose_y,Ypos_hat_ekf,Ypos_P_ekf,period)
		rov_pose.y = Ypos_hat_ekf[0][0]
		#(END) filtering the estimated positions
		#----------------------------------------------------------------------------------
		#(START) publishing the estimated 2D position
		pub_RelPose.publish(rov_pose)
		print("Relative position of rover to the anchor: %.9f -- x:%.6f -- y:%.6f" %
			(rospy.get_time(),rov_pose.x,rov_pose.y))
		Results_File_TwoTags_Pose.write("%.9f %.2f %.2f\r\n" % (rospy.get_time(),rov_pose.x,rov_pose.y))
		Results_File_TwoTags_Dists.write("%.9f %.2f %.2f\r\n" % (rospy.get_time(),DIST_tag1_flt,DIST_tag2_flt))
		#(END) publishing the estimated 2D position
		#----------------------------------------------------------------------------------

	rospy.spin()


if __name__ == '__main__':
	try:
		tag1()
	except (rospy.ROSInterruptException, rospy.ServiceException, rospy.ROSException):
		pass



