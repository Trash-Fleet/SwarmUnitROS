#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Int64
from geometry_msgs.msg import Pose

global counter
counter = 0

def callback(msg):
    global counter
    counter += 1

    t = rospy.Time().now()
    diff = t.to_nsec() - msg.data
    rospy.loginfo("subscriber %d %s" % (counter, t.to_nsec()))
    rospy.loginfo("diff: %d nsec" % (diff))
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("chatter", Int64, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()