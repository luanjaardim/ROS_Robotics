#!/usr/bin/env python3

import rospy
import RPi.GPIO as GPIO
from std_msgs.msg import Int16

RATE_HZ = 100

class MotorsNode:
    def __init__(self):
        rospy.init_node('motors_node')
        self.power_sub = rospy.Subscriber('power', Int16)

        self.PIN1 = int(rospy.get_param("~PIN1"))
        self.PIN2 = int(rospy.get_param("~PIN2"))
        GPIO.setup(self.CH1, GPIO.IN)
        GPIO.setup(self.CH2, GPIO.IN)

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.CH1, GPIO.IN)
        GPIO.setup(self.CH2, GPIO.IN)
        rospy.on_shutdown(GPIO.cleanup)

    def run(self):
        rate = rospy.Rate(RATE_HZ)
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    node = MotorsNode()
    node.run()
