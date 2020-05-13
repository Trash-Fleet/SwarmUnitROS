#!/usr/bin/env python
# license removed for brevity
import rospy
import serial
import sys
from std_msgs.msg import Int32, Float32
from sensor_msgs.msg import Imu

SERIAL_BAUD = 115200
SERIAL_PORT = "/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_55330333930351819242-if00"

class MotorParserNode(object):
    def __init__(self):
        self.motor1_vel = 0
        self.motor2_vel = 0
        
        self.ser = serial.Serial()
        self.ser.baudrate = SERIAL_BAUD
        self.ser.port = SERIAL_PORT
        self.ser.open()

        rospy.sleep(5)

        # send default pid values
        self.ser.write(b"P%f,I%f,D%f\n" %(1000, 30, 0)) # left
        self.ser.write(b"p%f,i%f,d%f\n" %(1000, 30, 0)) # right

        self.motor1_vel_pub = rospy.Publisher('motor1/fb/vel', Float32, queue_size=10)
        self.motor1_enc_pub = rospy.Publisher('motor1/fb/enc', Int32, queue_size=10)
        rospy.Subscriber("motor1/cmd/vel", Float32, self.motor1_vel_callback)
        rospy.Subscriber("motor1/cmd/p", Float32, self.motor1_p_callback)
        rospy.Subscriber("motor1/cmd/i", Float32, self.motor1_i_callback)
        rospy.Subscriber("motor1/cmd/d", Float32, self.motor1_d_callback)

        self.motor2_vel_pub = rospy.Publisher('motor2/fb/vel', Float32, queue_size=10)
        self.motor2_enc_pub = rospy.Publisher('motor2/fb/enc', Int32, queue_size=10)
        rospy.Subscriber("motor2/cmd/vel", Float32, self.motor2_vel_callback)
        rospy.Subscriber("motor2/cmd/p", Float32, self.motor2_p_callback)
        rospy.Subscriber("motor2/cmd/i", Float32, self.motor2_i_callback)
        rospy.Subscriber("motor2/cmd/d", Float32, self.motor2_d_callback)

        self.imu_pub = rospy.Publisher('imu', Imu, queue_size=10)

        rospy.loginfo("Motor Parser Node intialized!")

        rospy.init_node('motor_parser', anonymous=True)
        rate = rospy.Rate(25)
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

                self.ser.write(b'L%f,R%f\n' % (self.motor1_vel, self.motor2_vel))
            except:
                continue
            rate.sleep()

    def motor1_vel_callback(self, msg):
        if msg.data == 0:
            self.motor1_vel = 0
        else:
            # scale motor 1 by linear offset
            self.motor1_vel = 0.12 + (msg.data - 0.1) * 1.1

    def motor1_p_callback(self, msg):
        self.ser.write(b'P%f\n' %(msg.data))

    def motor1_i_callback(self, msg):
        self.ser.write(b'I%f\n' %(msg.data))

    def motor1_d_callback(self, msg):
        self.ser.write(b'D%f\n' %(msg.data))

    def motor2_vel_callback(self, msg):
        self.motor2_vel = msg.data

    def motor2_p_callback(self, msg):
        self.ser.write(b'p%f\n' %(msg.data))

    def motor2_i_callback(self, msg):
        self.ser.write(b'i%f\n' %(msg.data))

    def motor2_d_callback(self, msg):
        self.ser.write(b'd%f\n' %(msg.data))

    def parse_message(self, msg):
        vals = msg.strip().split(',')
        return vals

if __name__ == '__main__':
    try:
        MotorParserNode()
    except rospy.ROSInterruptException:
        pass
