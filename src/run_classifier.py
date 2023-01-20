#!/usr/bin/env python3

import joblib
import rospy
from std_msgs.msg import Float64MultiArray, Bool
import numpy as np

class SlipPredictor:

    def __init__(self):

        self.network = joblib.load('/home/miranda/ros_ws/src/applehand1_slip_detection/src/detection_only_1.pkl')
        self.memory = []
        self.prediction = 0

    def update(self,msg):
        
        self.memory.extend(msg.data)
        
        if len(self.memory) > 300:
            self.memory = self.memory[-300:]
            self.out = self.network.predict(np.array(self.memory).reshape(1,-1))

        
if __name__ == '__main__':

    rospy.init_node('classifier')

    classifier = SlipPredictor()

    feature_subscriber = rospy.Subscriber('feature', Float64MultiArray, classifier.update)

    publisher = rospy.Publisher('prediction', Bool, queue_size=10)

    while not rospy.is_shutdown():

        publisher.publish(classifier.prediction)