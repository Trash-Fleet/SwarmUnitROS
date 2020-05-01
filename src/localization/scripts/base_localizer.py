#!/usr/bin/env python
import rospy
import sys
import math
import time
import numpy as np

import tf2_ros

from std_msgs.msg import Float64, Int32
from geometry_msgs.msg import Pose, TransformStamped
from sensor_msgs.msg import Imu

from tf.transformations import euler_from_quaternion, quaternion_from_euler

# ROS variables
vel_pub = None

# Constants
p = 0.2
d = 0
enc_tpr = 537.6
enc_radius = 0.05 # 
enc_ticks_to_m = 2.0 * math.pi * enc_radius / enc_tpr
wheel_dist = 0.34 # 34cm

# Calculation variables
left_enc = None
prev_left_enc = 0
right_enc = None
prev_right_enc = 0
imu_orientation = None
imu_quaternion = None
state = [0,0,0]

def enc1_callback(msg):
    global right_enc
    right_enc = msg.data

def enc2_callback(msg):
    global left_enc
    left_enc = msg.data

def imu_callback(msg):
    global imu_orientation, imu_quaternion

    x = msg.orientation.x
    y = msg.orientation.y
    z = msg.orientation.z
    w = msg.orientation.w

    imu_quaternion = [x, y, z, w]
    (roll, pitch, yaw) = euler_from_quaternion(imu_quaternion)
    imu_orientation = [roll, pitch, yaw]

def main():
    global right_enc, prev_right_enc
    global left_enc, prev_left_enc
    global imu_orientation, imu_quaternion
    global state

    rospy.init_node('dead_reckoning', anonymous=True)

    rospy.Subscriber("motor1/fb/enc", Int32, enc1_callback) # left
    rospy.Subscriber("motor2/fb/enc", Int32, enc2_callback) # right
    rospy.Subscriber("imu", Imu, imu_callback)

    pose_pub = rospy.Publisher("base/pose", Pose, queue_size=10)

    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        if right_enc == None or left_enc == None:
            continue

        if imu_orientation == None or imu_quaternion == None:
            continue

        if prev_right_enc == None or prev_left_enc == None:
            prev_right_enc = right_enc
            prev_left_enc = left_enc

        right_diff = right_enc - prev_right_enc
        left_diff = left_enc - prev_left_enc

        avg_diff = (right_diff + left_diff)/2.0

        prev_left_enc = left_enc
        prev_right_enc = right_enc

        roll = imu_orientation[0]
        pitch = imu_orientation[1]
        yaw = imu_orientation[2]

        yawMatrix = np.matrix([
                    [math.cos(yaw), -math.sin(yaw), 0],
                    [math.sin(yaw), math.cos(yaw), 0],
                    [0, 0, 1]
                    ])

        pitchMatrix = np.matrix([
                    [math.cos(pitch), 0, math.sin(pitch)],
                    [0, 1, 0],
                    [-math.sin(pitch), 0, math.cos(pitch)]
                    ])

        rollMatrix = np.matrix([
                    [1, 0, 0],
                    [0, math.cos(roll), -math.sin(roll)],
                    [0, math.sin(roll), math.cos(roll)]
                    ])

        R = yawMatrix * pitchMatrix * rollMatrix
        vel_mat = np.matrix([[avg_diff], [0], [0]])

        transf_vel = R * vel_mat

        state[0] = state[0] + transf_vel[0,0]*enc_ticks_to_m
        state[1] = state[1] + transf_vel[1,0]*enc_ticks_to_m
        state[2] = state[2] + transf_vel[2,0]*enc_ticks_to_m

        # send transform message to ROS
        broadcaster = tf2_ros.StaticTransformBroadcaster()
        tf_msg = TransformStamped()

        tf_msg.header.stamp = rospy.Time.now()
        tf_msg.header.frame_id = "odom"
        tf_msg.child_frame_id = "base_link"

        tf_msg.transform.translation.x = state[0]
        tf_msg.transform.translation.y = state[1]
        tf_msg.transform.translation.z = state[2]
        tf_msg.transform.rotation.x = imu_quaternion[0]
        tf_msg.transform.rotation.y = imu_quaternion[1]
        tf_msg.transform.rotation.z = imu_quaternion[2]
        tf_msg.transform.rotation.w = imu_quaternion[3]

        broadcaster.sendTransform(tf_msg)

        # send Pose message to ROS on /base/pose
        new_msg = Pose()
        new_msg.position.x = state[0]
        new_msg.position.y = state[1]
        new_msg.position.z = 0

        new_msg.orientation.x = imu_quaternion[0]
        new_msg.orientation.y = imu_quaternion[1]
        new_msg.orientation.z = imu_quaternion[2]
        new_msg.orientation.w = imu_quaternion[3]

        pose_pub.publish(new_msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
