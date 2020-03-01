#!/usr/bin/env python
# license removed for brevity
import rospy
import serial
import sys
from std_msgs.msg import Int32, Float32
from sensor_msgs.msg import Imu

SERIAL_BAUD = 115200
SERIAL_PORT = "/dev/ttyACM0"

class MotorParserNode(object):
    def __init__(self):
        self.motor1_vel = 0
        self.motor2_vel = 0
        
        self.ser = serial.Serial()
        self.ser.baudrate = SERIAL_BAUD
        self.ser.port = SERIAL_PORT
        self.ser.open()

        self.motor1_vel_pub = rospy.Publisher('motor1/fb/vel', Float32, queue_size=10)
        self.motor1_enc_pub = rospy.Publisher('motor1/fb/enc', Int32, queue_size=10)
        rospy.Subscriber("motor1/cmd/vel", Float32, self.motor1_vel_callback)

        self.motor2_vel_pub = rospy.Publisher('motor2/fb/vel', Float32, queue_size=10)
        self.motor2_enc_pub = rospy.Publisher('motor2/fb/enc', Int32, queue_size=10)
        rospy.Subscriber("motor2/cmd/vel", Float32, self.motor2_vel_callback)

        self.imu_pub = rospy.Publisher('imu', Imu, queue_size=10)

        rospy.init_node('motor_parser', anonymous=True)
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            try:
                msg = self.ser.readline()
                parsed = msg.decode("utf-8")
                vals = self.parse_message(parsed)

                self.motor1_enc_pub.publish(int(vals[0]))
                self.motor2_enc_pub.publish(int(vals[1]))
                self.motor1_vel_pub.publish(float(vals[2]))
                self.motor2_vel_pub.publish(float(vals[3]))

                imu_msg = Imu()
                imu_msg.orientation.w = float(vals[4])
                imu_msg.orientation.x = float(vals[5])
                imu_msg.orientation.y = float(vals[6])
                imu_msg.orientation.z = float(vals[7])
                self.imu_pub.publish(imu_msg)

                self.ser.write(b'%f,%f\n' % (self.motor1_vel, self.motor2_vel))
            except:
                continue
            rate.sleep()

    def motor1_vel_callback(self, msg):
        self.motor1_vel = msg.data

    def motor2_vel_callback(self, msg):
        self.motor2_vel = msg.data

    def parse_message(self, msg):
        vals = msg.strip().split(',')
        return vals

if __name__ == '__main__':
    try:
        MotorParserNode()
    except rospy.ROSInterruptException:
        pass
