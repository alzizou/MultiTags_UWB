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

serial_inst = serial.Serial("/dev/ttyS0")
serial_inst.baudrate = 115200

This_tag = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
This_tag.bind(('192.168.3.184',8100))
This_tag.settimeout(0.1)

Other_tag = ('192.168.3.183',8100)

anchor_str = 'E'
MEAS = {0,0}
DATA_Store = np.zeros([2,2])
DIST_tag1 = 0

class Anchor:
    x = 0.0
    y = 0.0
    z = 0.0

anchor_pos = Anchor()
anchor_pos.x = 0.0
anchor_pos.y = 0.0
anchor_pos.z = 1200.0

Rover_Z_pos = 100.0
Rel_dist_tags = 400.0
rov_yaw = 0.0
rov_pose = Vector3() #mm
rov_pose.z = Rover_Z_pos

def clean_shutdown():
	serial_inst.close()

def callback_quat(msg):
	global rov_yaw
	siny_cosp = (2.0 * (msg.w * msg.z + msg.x * msg.y))
	cosy_cosp = (1.0 - 2.0 * (msg.y * msg.y + msg.z * msg.z))
	rov_yaw_rad = mt.atan2(siny_cosp, cosy_cosp)
	rov_yaw

def tag1():

	global anchor_str, DATA, DIST_tag1, DIST_tag2, jsn_data_tag2
	global anchor_pos, Rover_Z_pos, Rel_dist_tags, rov_pose, rov_yaw

	time_stmp = 0.0
	rospy.init_node("tag1",anonymous=True)
	pub_RelPose = rospy.Publisher("ROV_Rel_Pos", Vector3, queue_size=1)
	rate = rospy.Rate(100)
	rospy.on_shutdown(clean_shutdown)

	rospy.Subscriber("ROV_Quat", Quaternion, callback_quat)

	while not rospy.is_shutdown():
		rate.sleep()
		period = float(rospy.get_time()) - time_stmp
		time_stmp = float(rospy.get_time())
		freq = 1.0/period
		print("Frequency of tag1 node: %.2f" % freq)
		#----------------------------------------------------------------------------------
		#(START) measuring distance of tag1 to the anchor
		serial_inst.flush()
		temp_anch_str = None
		while(temp_anch_str==None):
			serial_inst.write(anchor_str.encode())
			temp_anch_str = serial_inst.readline()
		if(temp_anch_str!=None):
			MEAS = distance(serial_inst)
			if(MEAS[1] > 1):
				DATA_Store[0][0] = MEAS[0]
				DATA_Store[0][1] = MEAS[1]
			if(MEAS[1] <= 1):
				MEAS[0] = DATA_Store[0][0]
				MEAS[1] = DATA_Store[0][1]
			DIST_tag1 = MEAS[1]
			temp_anch_str = None
		print(colored(("Tag1 to Anchor Distance mm: %.6f %d \r\n" %
			(rospy.get_time(),DIST_tag1)),"green"))
		#(END) measuring distance of tag1 to the anchor
		#----------------------------------------------------------------------------------
		#(START) receiving distance of tag2 to the anchor
		try:
			This_tag.sendto('1'.encode(),Other_tag)
			Resp, Addrs = This_tag.recvfrom(1000)
			jsn_data_tag2 = json.loads(Resp.decode())
			DIST_tag2 = int(jsn_data_tag2.get("dist"))
			print(colored(("Tag2 to Anchor Distance mm: %.6f %d \r\n" %
				(rospy.get_time(),DIST_tag2)),"green"))
		except socket.timeout:
			print("No ditance received from Tag2!")
		#(END) receiving distance of tag2 to the anchor
		#----------------------------------------------------------------------------------
		#(START) computing the 2D position of rover
		projected_dist_tag1 = DIST_tag1 * mt.cos(anchor_pos.z - Rover_Z_pos)
		projected_dist_tag2 = DIST_tag2 * mt.cos(anchor_pos.z - Rover_Z_pos)
		longer_dist = projected_dist_tag1
		shorter_dist = projected_dist_tag2
		if (projected_dist_tag2 > longer_dist):
			longer_dist = projected_dist_tag2
			shorter_dist = projected_dist_tag1
		param_l0 = ( mt.pow(Rel_dist_tags,2) + mt.pow(longer_dist,2) - mt.pow(shorter_dist,2) ) / (2.0 * longer_dist)
		param_r0 = mt.sqrt( mt.pow(Rel_dist_tags,2) - mt.pow(param_l0,2) )
		param_theta = mt.atan2(param_r0 / param_l0)
		param_dist_avg = 0.5 * (longer_dist + shorter_dist)
		rov_pose.x = param_dist_avg * mt.cos(param_theta + rov_yaw)
		rov_pose.y = param_dist_avg * mt.sin(param_theta + rov_yaw)
		pub_RelPose.publish(rov_pose)
		print("Relative position of rover to the anchor: %.9f -- x:%.6f -- y:%.6f" %
			(rospy.get_time(),rov_pose.x,rov_pose.y))
		#(END) computing the 2D position of rover
		#----------------------------------------------------------------------------------

	rospy.spin()


if __name__ == '__main__':
	try:
		tag1()
	except (rospy.ROSInterruptException, rospy.ServiceException, rospy.ROSException):
		pass



