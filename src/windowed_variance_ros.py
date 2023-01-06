#!/usr/bin/env python3

import numpy as np
import rospy
from std_msgs.msg import Float64

class VarianceCalculator:

	def __init__(self):

		self.memory = []
		self.variance = 0.0
	
	def add_value(self,value):

		self.memory.append(value)
		if len(self.memory) > 5:
			self.memory = self.memory[-5:]

	def update(self, msg):
        
		value = msg.data
		self.add_value(value)
		self.variance = np.var(self.memory)

if __name__ == '__main__':

    rospy.init_node('variance_calculator')

    variance_calculator = VarianceCalculator()

    signal_subscriber = rospy.Subscriber('signal_topic', Float64, variance_calculator.update)

    filter_publisher = rospy.Publisher('variance_topic', Float64, queue_size=10)

    while not rospy.is_shutdown():

        filter_publisher.publish(variance_calculator.variance)
