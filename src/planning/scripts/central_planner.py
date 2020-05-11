#!/usr/bin/env python
import rospy
import sys
import time
import tf
import math

import numpy as np

from std_msgs.msg import Float32, Bool, String
from geometry_msgs.msg import Pose
from vision.yolo_opencv import detect_trash

from tf.transformations import euler_from_quaternion, quaternion_from_euler

class CentralPlanner(object):
    def __init__(self):
        ### Initialization##
        ####################

        # initialize attributes
        self.state = [0, 0, 0]
        self.search_angle_gap = 10
        self.base_traj_done = False
        self.arm_traj_done = False

        # initialize node
        rospy.init_node('central_planner', anonymous=True)

        # initialize ROS publishers and subscribers
        # Initialize subscribers
        rospy.Subscriber('base/done', Bool, self.base_done_cb)
        rospy.Subscriber('arm/done', Pose, self.arm_done_cb)

        # Initialize publishers
        self.base_target_pub = rospy.Publisher('/base/target_pose', Pose, queue_size=10)
        self.arm_target_pub = rospy.Publisher('/arm/target_pose', Pose, queue_size=10)
        self.arm_task_type_pub = rospy.Publisher('/arm/task', String, queue_size=10)

        # initialize tf listener
        self.listener = tf.TransformListener()

        # initialize base planner, arm planner
        rospy.sleep(1)

        ### Main sequence###
        ####################
        
        # Search for trash and go to the trash
        self.search_trash()
        self.locate_trash() #Need to wrie

        # Grab trash
        self.manipulate_trash("grab")

        # Go back to home
        self.call_base_action([0,0,0])

        # Drop trash 
        self.trash_location = #Drop the location fo the trash
        self.manipulate_trash("drop")

        # Stop
        print("Mission Complete!")
        
    def base_done_cb(self, msg):
        self.base_traj_done = True

    def arm_done_cb(self, msg):
        self.arm_traj_done = True
    
    def call_base_action(self, des_state):
        self.base_traj_done = False

        ang_quat = quaternion_from_euler(0, 0, des_state[2])

        pose_msg = Pose()

        pose_msg.position.x = des_state[0]
        pose_msg.position.y = des_state[1]
        pose_msg.position.z = 0
        pose_msg.orientation.x = ang_quat[0]
        pose_msg.orientation.y = ang_quat[1]
        pose_msg.orientation.z = ang_quat[2]
        pose_msg.orientation.w = ang_quat[3]

        self.base_target_pub.publish(pose_msg)
        print(pose_msg)

        while self.base_traj_done == False:
            continue

    def pose_callback(self, msg):
	# get transform information
        try:
            (trans,rot) = self.listener.lookupTransform('/odom', '/base_link', rospy.Time(0))

            self.state[0] = trans[0]
            self.state[1] = trans[1]

            (_,_,yaw) = euler_from_quaternion(rot)
            self.state[2] = yaw
#            print(self.state)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

    def camera_frame_transform(self, x, y):
        yawMatrix = np.matrix([
        [math.cos(self.state[2]), -math.sin(self.state[2]), 0, self.state[0]],
        [math.sin(self.state[2]), math.cos(self.state[2]), 0, self.state[1]],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
        ])
        
        originalMatrix = np.matrix([
        [1, 0, 0, x],
        [0, 1, 0, y],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
        ])

        result = yawMatrix * originalMatrix
        return [result[0,3], result[1,3]]

    def search_trash(self):
        search_angle = 0
        search_angle_degree =0
        detect_result = []

        # Turn around until a trash is detected
        while(len(detect_result) == 0):
            self.call_base_action([0,0,search_angle])
            search_angle_degree = search_angle_degree + self.search_angle_gap
            print(search_angle_degree)
            search_angle = float(search_angle_degree % 360) / 180.0 * math.pi
            print(search_angle)
            detect_result = detect_trash()
            print(detect_result)

        # Post the trash location to the base planner
        target_position = self.camera_frame_transform(detect_result[0][2],-detect_result[0][0])
        self.call_base_action([target_position[0],target_position[1], self.state[2]])

    def locate_trash(self):
        return 0

    def manipulate_trash(self, task):
        self.arm_task_type_pub.publish(task)

        # Wait for the robot to finish its movement
        while(1):
            if(base_done_cb == True):
                base_done_cb = False
                break

        # Publish the pose trash
        pose_msg = Pose()

        pose_msg.position.x = self.trash_location[0]
        pose_msg.position.y = self.trash_location[1]
        pose_msg.position.z = self.trash_location[2]
        pose_msg.orientation.x = 0
        pose_msg.orientation.y = 0
        pose_msg.orientation.z = 0
        pose_msg.orientation.w = 0

        self.arm_target_pub.publish(pose_msg)

        # Wait for arm to finish its movement
        while(1):
            if(arm_traj_done == True):
                arm_traj_done = False
                break

    

if __name__ == '__main__':
    try:
        # Start the central planner
        CentralPlanner()
    except rospy.ROSInterruptException:
        pass
 