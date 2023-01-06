#!/usr/bin/env python3

import numpy as np
import rospy
from std_msgs.msg import Float64, Float64MultiArray

class FeatureStacker:

	def __init__(self):

		self.feature = [0, 0, 0, 0, 0, 0]


	def update(self, msg, idx):
        
		self.feature[idx]=msg.data

if __name__ == '__main__':

    rospy.init_node('accelerometer_filter')

    stacker = FeatureStacker()

    az1_subscriber = rospy.Subscriber('az1_var', Float64, stacker.update,0)

    feature_publisher = rospy.Publisher('feature', Float64MultiArray, queue_size=10)

    while not rospy.is_shutdown():

        feature_out = Float64MultiArray
        feature_out.data = stacker.feature
        feature_publisher.publish(feature_out)