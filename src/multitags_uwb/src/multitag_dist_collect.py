#!/usr/bin/env python3
import rospy
import serial
import numpy as np
import socket
import json
from tag_dist import tag_dist

Anchor_Str = 'E'
Total_Tags_Num = 3

All_Data = {}
All_Serial_Inst = [None]*Total_Tags_Num
All_Tag_Dist = [None]*Total_Tags_Num
for i in range(0,Total_Tags_Num):
	All_Serial_Inst[i] = serial.Serial(("/dev/ttyS%d"%i), baudrate=115200)
	Tag_ID = 100*(i+1)
	All_Tag_Dist[i] = tag_dist(Tag_ID, serial_inst, Anchor_Str)


def clean_shutdown():
	global All_Serial_Inst
	for i in range(0,Total_Tags_Num):
		All_Serial_Inst[i].close()
	print("All serial ports to the tags are closed successfully!")


def main():
	global All_Data, Total_Tags_Num, All_Serial_Inst, Tag_Dist

	time_stmp = 0.0
	rospy.init_node("multitag_dist_collect", anonymous=True)
	pub_tag_dist = rospy.Publisher("Tags_Distance", String, queue_size = 1)
	rate = rospy.Rate(100)
	rospy.on_shutdown(clean_shutdown)
	serial_inst.close()
	json_obj = String()

	while not rospy.is_shutdown():
		rate.sleep()
		period = float(rospy.get_time()) - time_stmp
		time_stmp = float(rospy.get_time())
		freq = 1.0/period
		#-----------------------------------------------------------------------------------
		#(START) measuring and filtering distance of all the tags to the only anchor
		for i in range(0,Total_Tags_Num):
			All_Tag_Dist[i].dist_collect()
			All_Tag_Dist[i].dist_filter()
		#(END) measuring and filtering distance of all the tags to the only anchor
		#-----------------------------------------------------------------------------------
		#(START) publishing the measured distances of tags to the anchor
		for i in range(0,Total_Tags_Num):
			All_Data[i] = {All_Tag_Dist[i].Tag_ID,All_Tag_Dist[i].filt_dist}
		json_obj.data = json.dumps(All_Data)
		pub_tag_dist.publish(json_obj)
		#(END) publishing the measured distances of tags to the anchor
		#-----------------------------------------------------------------------------------


if __name__ == __main__:
	try:
		main()
	except (rospy.ROSInterruptException, rospy.ServiceException, rospy.ROSException):
		pass
