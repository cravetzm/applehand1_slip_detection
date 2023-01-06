#!/usr/bin/env python3

import numpy as np
import rospy
from applehand.msg import ImuNaive
from std_msgs.msg import Float64
from scipy.signal import firwin, lfilter

class AccelFilter:

	def __init__(self):

		self.memory = []
		self.filtered_value = 0.0
		self.filter = firwin(167, 10, fs=70, pass_zero=False)
	
	def add_value(self,value):

		self.memory.append(value)

	def filter_update(self, msg):
		value = msg.linear_acceleration.z
		self.add_value(value)
		self.filtered_value = lfilter(self.filter, 1.0, self.memory)[-1]

if __name__ == '__main__':

    rospy.init_node('accelerometer_filter')

    imu_filter = AccelFilter()

    imu_subscriber = rospy.Subscriber('imu_topic', ImuNaive, imu_filter.filter_update)

    filter_publisher = rospy.Publisher('filtered_accelerometer_topic', Float64, queue_size=10)

    while not rospy.is_shutdown():

        filter_publisher.publish(imu_filter.filtered_value)
