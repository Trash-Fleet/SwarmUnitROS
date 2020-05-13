#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String, Int64
from geometry_msgs.msg import Pose

def talker():
    pub = rospy.Publisher('chatter', Int64, queue_size=10)
    rospy.init_node('talker', anonymous=True)

    counter = -1

    rate = rospy.Rate(2) # 10hz
    while not rospy.is_shutdown():
        counter += 1
        t = rospy.Time().now()
        hello_str = "publisher %d %s" % (counter, t.to_nsec())
        rospy.loginfo(hello_str)

        pose_msg = Pose()
        
        pub.publish(t.to_nsec())
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass