#!/usr/bin/env python
import rospy
import sys
import time
import tf
import math
import copy

import numpy as np

from std_msgs.msg import Float64, Bool, String
from geometry_msgs.msg import Pose
from vision.yolo_opencv import Yolo

from tf.transformations import euler_from_quaternion, quaternion_from_euler

class CentralPlanner(object):
    def __init__(self):
        ### Initialization##
        ####################

        # initialize attributes
        self.state = [0, 0, 0]
        self.search_angle_gap = -45
        self.base_traj_done = False
        self.arm_traj_done = False
        self.yolo = Yolo()

        # initialize node
        rospy.init_node('central_planner', anonymous=True)

        # initialize ROS publishers and subscribers
        # Initialize subscribers
        rospy.Subscriber('base/pose', Pose, self.base_pose_cb)
        rospy.Subscriber('base/done', Bool, self.base_done_cb)
        rospy.Subscriber('arm/done', Bool, self.arm_done_cb)

        # Initialize publishers
        self.base_target_pub = rospy.Publisher('/base/target_pose', Pose, queue_size=10)
        self.arm_target_pub = rospy.Publisher('/arm/target_pose', Pose, queue_size=10)
        self.arm_task_type_pub = rospy.Publisher('/arm/task', String, queue_size=10)
        self.gripper_joint_pub = rospy.Publisher('/gripper_joint/command', Float64, queue_size=10)

        # initialize tf listener
        self.listener = tf.TransformListener()

        # initialize base planner, arm planner
        rospy.sleep(2)

        ### Main sequence###
        ####################

        # self.call_base_action([0.5, 0, 0])
        # self.call_base_action([0, 0, 0])

        # set robot arm to home position
        self.arm_task_type_pub.publish('home')
        
        # Search for trash and go to the trash
        print("starting search")
        trash_pos_cam = self.search_trash()
        print("found target")
        print(trash_pos_cam)

        # trash_pos_cam = self.locate_trash()
        
        # determine trash position in robot frame
        trash_pos_odom = self.camera_to_odom_transform(trash_pos_cam[0], trash_pos_cam[1])

        # alter trash position by negative offset
        offset = 0.30
        trash_pos_odom[0] -= offset * np.cos(self.state[2])
        trash_pos_odom[1] -= offset * np.sin(self.state[2])

        # move to shifted base position
        print("going toward target")
        print(trash_pos_odom)
        self.call_base_action([trash_pos_odom[0],trash_pos_odom[1], self.state[2]])

        # look for trash again
        trash_pos_cam = self.locate_trash()
        trash_pos_odom = self.camera_to_odom_transform(trash_pos_cam[0], trash_pos_cam[1])

        counter = 0
        while np.linalg.norm(trash_pos_cam) == 0 and counter < 10:
            cur_state = copy.deepcopy(self.state)
            self.call_base_action([cur_state[0], cur_state[1], cur_state[2] + 0.20])
            self.call_base_action([cur_state[0], cur_state[1], cur_state[2]])

            trash_pos_cam = self.locate_trash()
            counter += 1

        # replan base trajectory if base is too far from detected trash
        if np.linalg.norm(trash_pos_cam > offset):
            # alter trash position by negative offset
            offset = 0.30
            trash_pos_odom[0] -= offset * np.cos(self.state[2])
            trash_pos_odom[1] -= offset * np.sin(self.state[2])

            # move to shifted base position
            print("going toward target")
            print(trash_pos_odom)
            self.call_base_action([trash_pos_odom[0],trash_pos_odom[1], self.state[2]])
        
        # regrab if necessary
        regrab_counter = 0
        while trash_pos_cam != [] and np.linalg.norm(trash_pos_cam) > 0:
            # make sure gripper is open c: 
            self.gripper_joint_pub.publish(0.0)
            rospy.sleep(2)
            
            print("regrab attempt: %d" %(regrab_counter))

            # determine trash position in robot frame
            trash_pos_arm = self.camera_to_arm_transform(trash_pos_cam[0], trash_pos_cam[1])
            print("trash position: ")
            print(trash_pos_arm)

            # account for bottle radius
            bottle_radius = 0.01
            yaw = np.arctan2(trash_pos_arm[1], trash_pos_arm[0])
            trash_pos_arm[0] += bottle_radius * np.cos(yaw)
            trash_pos_arm[1] += bottle_radius * np.sin(yaw)

            print("grabbing trash")
            self.manipulate_trash('grab', trash_pos_arm)
            
            cur_state = copy.deepcopy(self.state)
            self.call_base_action([cur_state[0], cur_state[1], cur_state[2] + 0.20])
            self.call_base_action([cur_state[0], cur_state[1], cur_state[2]])

            trash_pos_cam = self.locate_trash()

            counter = 0
            while trash_pos_cam != [] and np.linalg.norm(trash_pos_cam) == 0 and counter < 10:
                cur_state = copy.deepcopy(self.state)
                self.call_base_action([cur_state[0], cur_state[1], cur_state[2] + 0.20])
                self.call_base_action([cur_state[0], cur_state[1], cur_state[2]])
                trash_pos_cam = self.locate_trash()
                counter += 1

            regrab_counter += 1


        # # Go back to home
        self.call_base_action([-0.1,0,np.pi])

        # # Drop trash 
        # self.manipulate_trash("drop", [0, 0.2])
        self.gripper_joint_pub.publish(0.0)
        rospy.sleep(2)

        # go to home position
        self.arm_task_type_pub.publish('home')

        # Stop
        print("Mission Complete!")

    ## ROS CALLBACK FUNCTIONS
        
    def base_done_cb(self, msg):
        self.base_traj_done = True

    def arm_done_cb(self, msg):
        print("received arm_done")
        self.arm_traj_done = True

    def base_pose_cb(self, msg):
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

    ## HELPER FUNCTIONS 
    
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

    def camera_to_odom_transform(self, x, y):
        R_base_to_odom = np.matrix([
            [math.cos(self.state[2]), -math.sin(self.state[2]), 0, self.state[0]],
            [math.sin(self.state[2]), math.cos(self.state[2]), 0, self.state[1]],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        R_cam_to_base = np.matrix([
            [np.cos(0), -np.sin(0), 0, 0.05],
            [np.sin(0), np.cos(0), 0, 0.02],
            [0, 0, 1, 0.04],
            [0, 0, 0, 1]
        ])
        
        originalMatrix = np.matrix([
            [1, 0, 0, x],
            [0, 1, 0, y],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        result = R_base_to_odom * R_cam_to_base * originalMatrix
        return [result[0,3], result[1,3]]

    def camera_to_arm_transform(self, x, y):
        R_cam_to_base = np.eye(4)

        R_cam_to_base = R_cam_to_base.dot(
            np.matrix([
                [np.cos(0), -np.sin(0), 0, 0.05],
                [np.sin(0), np.cos(0), 0, 0.02],
                [0, 0, 1, 0.04],
                [0, 0, 0, 1]
            ])
        )

        R_arm_to_base = np.eye(4)
        R_arm_to_base = R_arm_to_base.dot(
            np.matrix([
                [np.cos(-np.pi/2.0), -np.sin(-np.pi/2.0), 0, 0.0],
                [np.sin(-np.pi/2.0), np.cos(-np.pi/2.0), 0, 0.0],
                [0, 0, 1, 0.17],
                [0, 0, 0, 1]
            ])
        )

        R_cam_to_arm = np.linalg.inv(R_arm_to_base).dot(R_cam_to_base)
        h = np.array([x, y, 0, 1]).reshape(-1,1)

        result = R_cam_to_arm.dot(h).reshape(-1)

        return [result[0,0], result[0,1], result[0,2]]

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
            search_angle = np.arctan2(np.sin(search_angle), np.cos(search_angle))
            print(search_angle)
            # detect_result = self.yolo.detect_trash()
            detect_result = self.locate_trash()
            print(detect_result)

        return detect_result

    def locate_trash(self):
        detect_result = []

        try:
            detect_result = self.yolo.detect_trash()
        except:
            return detect_result

        if len(detect_result) == 0: return detect_result

        # find closest point
        closest_i = -1
        closest_norm = 100000
        for i in range(len(detect_result)):
            norm = np.linalg.norm(detect_result[i])
            if norm > 0 and norm < 3 and norm < closest_norm:
                closest_i = i

        trash_pos_cam = [detect_result[closest_i][2], -detect_result[closest_i][0]]
        return trash_pos_cam

    def manipulate_trash(self, task, trash_location):
        self.arm_traj_done = False

        # publish task
        self.arm_task_type_pub.publish(task)
        rospy.sleep(2)

        # Publish the pose trash
        pose_msg = Pose()
        pose_msg.position.x = trash_location[0]
        pose_msg.position.y = trash_location[1]
        pose_msg.position.z = 0.085
        pose_msg.orientation.x = 0
        pose_msg.orientation.y = 0
        pose_msg.orientation.z = 0
        pose_msg.orientation.w = 0
        self.arm_target_pub.publish(pose_msg)

        # Wait for arm to finish its movement
        while(self.arm_traj_done == False):
            continue
        
        # rospy.sleep(5)
        # while(1):
        #     if(arm_traj_done == True):
        #         arm_traj_done = False
        #         break    

if __name__ == '__main__':
    try:
        # Start the central planner
        CentralPlanner()
    except rospy.ROSInterruptException:
        pass
 
