#!/usr/bin/env python3

import numpy as np
import rospy
from applehand.msg import ImuNaive
from std_msgs.msg import Float64

class ImuFilter:

	def __init__(self):

		self.memory = []
		self.filtered_value = 0.0
	
	def add_value(self,value):

		self.memory.append(value)
		if len(self.memory) > 5:
			self.memory = self.memory[-5:]

	def filter_update(self, msg):
		value = msg.angular_velocity.x
		self.add_value(value)
		self.filtered_value = np.mean(self.memory)

if __name__ == '__main__':

    rospy.init_node('gyroscope_filter')

    imu_filter = ImuFilter()

    imu_subscriber = rospy.Subscriber('imu_topic', ImuNaive, imu_filter.filter_update)

    filter_publisher = rospy.Publisher('filtered_gyroscope_topic', Float64, queue_size=10)

    while not rospy.is_shutdown():

        filter_publisher.publish(imu_filter.filtered_value)
