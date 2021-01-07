#!/usr/bin/env python3
import rospy
import socket
import serial
import json
import numpy as np
import math as mt
from std_msgs.msg import String
from termcolor import colored
from distance import distance
from ekf_dist import EKF_DIST

serial_inst = serial.Serial("/dev/ttyS0")
serial_inst.baudrate = 115200

This_tag = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
This_tag.bind(('192.168.3.226',8100))
#This_tag.bind(('10.11.1.21',8100))
This_tag.settimeout(0.01)

Other_tag = ('192.168.3.184',8100)
#Other_tag = ('10.11.1.17',8100)

data_recv = '0'

anchor_str = ['C','E']
temp_anch_str = None
MEAS = {0,0}
DATA_Store = np.zeros([2,2])
DIST_tag2 = 0
jsn_data_tag2 = String()

x_hat_ekf = np.zeros([3,1])
P_ekf = np.eye(3)

def clean_shutdown():
	serial_inst.close()

def tag2():

	global temp_anch_str, data_recv, jsn_data_tag2, DIST_tag2
	global MEAS, DATA_Store, anchor_str
	global x_hat_ekf, P_ekf, DIST_tag2_flt

	time_stmp = 0.0
	rospy.init_node("tag2", anonymous=True)
	rate = rospy.Rate(100)
	rospy.on_shutdown(clean_shutdown)
	serial_inst.close()

	while not rospy.is_shutdown():
		rate.sleep()
		period = float(rospy.get_time()) - time_stmp
		time_stmp = float(rospy.get_time())
		freq = 1.0/period
		print("Frequency of tag2 node: %.2f" % freq)
		#----------------------------------------------------------------------------------
		try:
			Comnd, Addrs = This_tag.recvfrom(1000)
			data_recv = Comnd.decode()
			print(data_recv)
			if (data_recv == '1'):
				#----------------------------------------------------------------------------------
				#(START) measuring distance of tag2 to the anchor
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
					DIST_tag2 = MEAS[1]
					temp_anch_str = None
				print(colored(("Tag2 to Anchor Distance mm: %.6f %d \r\n" %
					(rospy.get_time(),DIST_tag2)),"green"))
				serial_inst.close()
				#(END) measuring distance of tag2 to the anchor
				#----------------------------------------------------------------------------------
				#(START) filtering the measured distance
				x_hat_ekf,P_ekf = EKF_DIST(DIST_tag2,x_hat_ekf,P_ekf,period)
				DIST_tag2_flt = x_hat_ekf[0][0]
				#(END) filtering the measured distance
				#----------------------------------------------------------------------------------
				#(START) sending the measured distance to tag1
				jsn_data_tag2 = json.dumps({"dist":DIST_tag2_flt})
				This_tag.sendto(jsn_data_tag2.encode(),Other_tag)
				data_recv = '0'
				#(END) sending the measured distance to tag2
				#----------------------------------------------------------------------------------
		except socket.timeout:
			print("No command received from tag1 to start distance measurement process!")



if __name__ == '__main__':
	try:
		tag2()
	except (rospy.ROSInterruptException, rospy.ServiceException, rospy.ROSException):
		pass
