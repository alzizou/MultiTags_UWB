#!/usr/bin/env python3
import rospy
import serial
import json
import numpy as np
import math as mt
from pymavlink import mavutil
from std_msgs.msg import String
from geometry_msgs.msg import Quaternion, Twist, Accel, Wrench
from nav_msgs.msg import Odometry
from termcolor import colored
from quaternion import quaternion

PX4_connection = mavutil.mavlink_connection("/dev/ttyACM0",115200)
PX4_connection.wait_heartbeat()
print(colored(("PX4 Heartbeat(%u,%u,%u,%s,%u)" % (PX4_connection.mav_type, PX4_connection.target_system, PX4_connection.target_component, PX4_connection.flightmode, PX4_connection.base_mode)),"yellow"))

Results_File_PX4Orient = open(r"/root/logs/Results_File_PX4Orient.txt","w+")
Results_File_PX4Accel = open(r"/root/logs/Results_File_PX4Accel.txt","w+")
Results_File_PX4Twist = open(r"/root/logs/Results_File_PX4Twist.txt","w+")
Results_File_PX4_Cont = open(r"/root/logs/Results_File_PX4_Cont.txt","w+")

Received_cont = False
Rov_des_Spd = 0.0
Rov_des_YawRate = 0.0

time_stmp = 0.0
data_PX4Orient = Quaternion()
data_PX4Twist = Twist()
data_PX4Accel = Accel()
controls_set = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]

def clean_shutdown():
	Results_File_PX4Orient.close()
	Results_File_PX4Accel.close()
	Results_File_PX4Twist.close()
	current_state_msg = PX4_connection.recv_match(type='HEARTBEAT',blocking=True)
	while(current_state_msg.system_status is not 3):
		PX4_connection.mav.command_long_send(PX4_connection.target_system,PX4_connection.target_component,mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,0,0,0,0,0,0,0,0)
		current_state_msg = PX4_connection.recv_match(type='HEARTBEAT',blocking=True)
	print(colored("Disarmed.","yellow"))
	print(colored("The connection to the PX4 is terminated!","yellow"))

def callback_Con_StP(msg):
	global Received_cont
	global Rov_des_Spd
	global Rov_des_YawRate
	Received_cont = False
	Rov_des_Spd = 0.0
	Rov_des_YawRate = 0.0
	if ((abs(msg.linear.x) > 0.0) or (abs(msg.angular.z) > 0.0)):
		Received_cont = True
		Rov_des_Spd = msg.linear.x
		Rov_des_YawRate = msg.angular.z

def px4_com():

	global time_stmp, data_PX4Orient, data_PX4Accel, data_PX4Twist, PX4_ATT_msg, PX4_ACC_msg
	global Spd_stp, YawRate_stp, Received_cont, controls_set, Result_Set
	global current_state_msg, PX4_connection, num_arm, Result_ARM, Result_Mod_Chng, num_mode, PX4_msg2

	rospy.init_node("px4_communication", anonymous=True)
	rate = rospy.Rate(100)
	rospy.on_shutdown(clean_shutdown)
	rospy.Subscriber("cmd_vel", Twist, callback_Con_StP)
	pub_PX4Accel = rospy.Publisher("ROV_PX4_Accel", Accel, queue_size=1)
	pub_PX4Orient = rospy.Publisher("ROV_PX4_Orient", Quaternion, queue_size=1)
	pub_PX4Twist = rospy.Publisher("ROV_PX4_Twist", Twist, queue_size=1)
	while not rospy.is_shutdown():
		rate.sleep()
		Period = float(rospy.get_time()) - time_stmp
		Frequency = 1.0/Period
		print(colored(("PX4 frequency : %0.2f" % Frequency),"yellow"))
		time_stmp = float(rospy.get_time())
		#--------------------------------------------------------------------------------------------------------------------------
		#(START) publishing PX4 measured attitude data
		PX4_ATT_msg = PX4_connection.recv_match(type='ATTITUDE_QUATERNION',blocking=True) #ATTITUDE
		if not PX4_ATT_msg:
			print(colored("PX4 IMU data: no message is received!","red"))
		else:
			data_PX4Orient.w = PX4_ATT_msg.q1
			data_PX4Orient.x = PX4_ATT_msg.q2
			data_PX4Orient.y = PX4_ATT_msg.q3
			data_PX4Orient.z = PX4_ATT_msg.q4
			pub_PX4Orient.publish(data_PX4Orient)
			print(colored(("PX4 Received quaternions: %.6f , %.6f , %.6f , %.6f" % (data_PX4Orient.w,data_PX4Orient.x,data_PX4Orient.y,data_PX4Orient.z)),"yellow"))
			Results_File_PX4Orient.write("%.9f %.6f %.6f %.6f %.6f\r\n" % (rospy.get_time(),data_PX4Orient.w,data_PX4Orient.x,data_PX4Orient.y,data_PX4Orient.z))
		#(END) publishing PX4 measured attitude data
		#--------------------------------------------------------------------------------------------------------------------------

	rospy.spin()


if __name__ == '__main__':
	try:
		px4_com()
	except (rospy.ROSInterruptException, rospy.ServiceException, rospy.ROSException):
		pass

