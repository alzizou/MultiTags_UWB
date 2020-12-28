#!/usr/bin/env python3
import rospy
import socket
import serial
import json
import numpy as np
import math as mt
from distance import distance

serial_inst = serial.Serial("/dev/ttyS0")
serial_inst.baudrate = 115200

This_tag = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
This_tag.bind(('192.168.3.183',8100))
This_tag.settimeout(0.1)

Other_tag = ('192.168.3.184',8100)

data_recv = '0'

anchor_str = 'E'
MEAS = {0,0}
DATA_Store = np.zeros([2,2])
DIST_tag2 = 0

def clean_shutdown():
	serial_inst.close()

def tag2():

	global data_recv, jsn_data_tag2, DIST_tag2, MEAS, DATA_Store, anchor_str

	time_stmp = 0.0
	rospy.init_node("tag2", anonymous=True)
	rate = rospy.Rate(100)
	rospy.on_shutdown(clean_shutdown)

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
			if (data_recv == '1'):
				#----------------------------------------------------------------------------------
				#(START) measuring distance of tag2 to the anchor
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
					DIST_tag2 = MEAS[1]
					temp_anch_str = None
				print(colored(("Tag2 to Anchor Distance mm: %.6f %d \r\n" %
					(rospy.get_time(),DIST_tag2)),"green"))
				#(END) measuring distance of tag2 to the anchor
				#----------------------------------------------------------------------------------
				#(START) sending the measured distance to tag1
				jsn_data_tag2 = json.dumps({"dist":DIST_tag2})
				This_tag.sendto(json.data_tag2.encode(),Other_tag)
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
