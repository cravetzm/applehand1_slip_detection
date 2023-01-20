#!/usr/bin/env python3

import rospy
import os, sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String

import tf2_ros, tf2_geometry_msgs


class SlipDetectionStateMachine:

    def __init__(self):
        
        self.state = None
        self.slip_signal = False
        self.centering_signal = False
        self.picker = UR5eProxyApplePicking
        self.start_pose = None
    
    def listen_to_slip(self, msg):
        self.slip_signal = msg.data

    def check_centering(self, msg):
        self.centering_signal = msg.data

    def update_state(self):

        if self.state is None:

            #todo: run centering
            self.state = "centering"
            #wait for ros to figure out that centering is happening
            while self.centering_signal == False:
                pass

        if self.state == "centering" and self.centering_signal == False:

            close_hand_service()
            self.start_pose = self.picker.move_group.get_current_pose().pose
            self.state = "picking"
            self.picker.retrieve(0.20)

        if self.state == "picking" and self.slip_signal == True:

            user_choice = input("Event detected: press 'r' to regrasp or 'd' to deposit apple")
            if user_choice == "d":
                self.deposit_apple()
            elif user_choice == "r":
                self.state = "slipping"
                print ("initiating regrasp")
            else:
                print("invalid input -- regraspping")
                self.state = "slipping"


        if self.state == "slipping":
            
            self.return_to_start()
            open_hand_service()
            self.picker.retrieve(10)
            self.state = None

    def deposit_apple(self):

        open_hand_service()

    def return_to_start(self):
        self.picker.move_group.set_pose_target(self.start_pose)
        self.picker.move_group.go(wait=True)
        self.picker.move_group.stop()
        self.picker.move_group.clear_pose_targets()

## ... Hand Related Functions...

def close_hand_service():
    os.system("rosservice call /applehand/close_hand")

def open_hand_service():
    os.system("rosservice call /applehand/open_hand")

def relax_hand_service():
    os.system("rosservice call /applehand/relax_hand")

## ... Motion code (credit to Alejo V.L.) ... 

class UR5eProxyApplePicking:

    def __init__(self):

        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander("maniupulator")

        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)

        self.success_or_failure_publisher = rospy.Publisher('/success_or_failure', String, queue_size=20)


    def retrieve(self, distance):

            current_pose = self.move_group.get_current_pose().pose

            # --- Step 1: Read the pose from the "Base_link" into "Tool0"
            # **Assuming /tf2 topic is being broadcasted
            tf_buffer = tf2_ros.Buffer()
            listener = tf2_ros.TransformListener(tf_buffer)

            pose_stamped = tf2_geometry_msgs.PoseStamped()
            pose_stamped.pose = current_pose
            pose_stamped.header.frame_id = self.ref_frame
            pose_stamped.header.stamp = rospy.Time(0)

            try:
                # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
                pose_at_tool = tf_buffer.transform(pose_stamped, "palm", rospy.Duration(1))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                raise

            waypoints = []

            #add current pose to start of trajectory

            try:
                # ** It is important to wait for the listener to start listening. Hence, the rospy.Duration(1)
                pose_at_world = tf_buffer.transform(pose_at_tool, self.ref_frame, rospy.Duration(1))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                raise

            waypoints.append(pose_at_world.pose)
        


            #add desired pose to trajectory
            pose_at_tool.pose.position.z = pose_at_tool.pose.position.z - distance
            # --- Step 3: Convert the pose back into the "Base_Link" reference frame
            pose_at_tool.header.stamp = rospy.Time(0)

            try:
                # ** It is important to wait for the listener to start listening. Hence, the rospy.Duration(1)
                new_pose_at_world = tf_buffer.transform(pose_at_tool, self.ref_frame, rospy.Duration(1))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                raise

            waypoints.append(new_pose_at_world.pose)

            # --- Step 4: Finally, move to the new position
            #self.move_group.set_pose_target(new_pose_at_world.pose)
            plan, _ = self.move_group.compute_cartesian_path(waypoints, 0.01, 0)
            self.move_group.execute(plan, wait=True)
            #plan = self.move_group.go(wait=True)
            #self.move_group.stop()
            #self.move_group.clear_pose_targets()

            # Compare poses
            pose_goal = new_pose_at_world.pose
            current_pose = self.move_group.get_current_pose().pose

            success = all_close(pose_goal, current_pose, 0.01)

            return success

def all_close(goal, actual, tolerance):
    """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
    all_equal = True
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True


    