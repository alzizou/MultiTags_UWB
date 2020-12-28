#!/usr/bin/env python3
import rospy
import serial
import numpy as np
import math as mt
from pymavlink import mavutil
from geometry_msgs.msg import Quaternion
from termcolor import colored

PX4_connection = mavutil.mavlink_connection("/dev/ttyACM0",115200)
PX4_connection.wait_heartbeat()
print(colored(("PX4 Heartbeat(%u,%u,%u,%s,%u)" %
	(PX4_connection.mav_type, PX4_connection.target_system,
	PX4_connection.target_component, PX4_connection.flightmode, PX4_connection.base_mode)),"yellow"))

#Results_File_PX4Orient = open(r"/root/logs/Results_File_PX4Orient.txt","w+")

data_PX4Orient = Quaternion()

def clear_shutdown():
#	Results_File_PX4Orient.close()
	print("px4 communication is lost!")

def px4_com():

	global data_PX4Orient

	time_stmp = 0.0
	rospy.init_node("px4_com", anonymous=True)
	pub_PX4Orient = rospy.Publisher("Rov_Quat", Quaternion, queue_size=1)
	rate = rospy.Rate(100)
	rospy.on_shutdown(clear_shutdown)

	while not rospy.is_shutdown():
		rate.sleep()
		period = float(rospy.get_time()) - time_stmp
		time_stmp = float(rospy.get_time())
		freq = 1.0/period
		print("Frequency of PX4 communication node: %.2f" % freq)
		#------------------------------------------------------------------------
		#(START) measuring the attitude of rover
		PX4_ATT_msg = PX4_connection.recv_match(type='ATTITUDE_QUATERNION',blocking=True)
		if not PX4_ATT_msg:
			print(colored("PX4 IMU data: no message is received!","red"))
		else:
			data_PX4Orient.w = PX4_ATT_msg.q1
			data_PX4Orient.x = PX4_ATT_msg.q2
			data_PX4Orient.y = PX4_ATT_msg.q3
			data_PX4Orient.z = PX4_ATT_msg.q4
			pub_PX4Orient.publish(data_PX4Orient)
			print(colored(("PX4 Received quaternions: %.6f , %.6f , %.6f , %.6f" %
				(data_PX4Orient.w,data_PX4Orient.x,data_PX4Orient.y,data_PX4Orient.z))
				,"yellow"))
#			Results_File_PX4Orient.write("%.9f %.6f %.6f %.6f %.6f\r\n" %
#				(rospy.get_time(),data_PX4Orient.w,data_PX4Orient.x,data_PX4Orient.y,
#				data_PX4Orient.z))


if __name__ == '__main__':
	try:
		px4_com()
	except (rospy.ROSInterruptException, rospy.ServiceException, rospy.ROSException):
		pass
