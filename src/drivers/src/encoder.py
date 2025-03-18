#!/usr/bin/env python3

import rospy
import RPi.GPIO as GPIO
from std_msgs.msg import Float32

PULSES_PER_REV = 11
GEAR_REDUCTION = 46
RATE_HZ = 100

class EncoderNode:
    def __init__(self):
        rospy.init_node('encoder_node')
        self.speed_pub = rospy.Publisher('wheel_speed', Float32, queue_size=1)
        self.pulses_count = 0
        self.last_time = rospy.Time.now()
        GPIO.setmode(GPIO.BCM)
        self.CH1 = int(rospy.get_param("~CH1"))
        self.CH2 = int(rospy.get_param("~CH2"))
        GPIO.setup(self.CH1, GPIO.IN)
        GPIO.setup(self.CH2, GPIO.IN)
        GPIO.add_event_detect(self.CH1, GPIO.BOTH, callback=self._encoder_callback)
        rospy.on_shutdown(GPIO.cleanup)

    def _encoder_callback(self, channel):
        if GPIO.input(self.CH1) != GPIO.input(self.CH2):
            self.pulses_count += 1
        else:
            self.pulses_count -= 1

    def run(self):
        rate = rospy.Rate(RATE_HZ)
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            dt_seconds = (current_time - self.last_time).to_sec()
            self.last_time = current_time

            if dt_seconds <= 0:
                dt_seconds = 1 / RATE_HZ

            motor_revolutions = self.pulses_count / PULSES_PER_REV
            self.pulses_count = 0
            
            output_revolutions = motor_revolutions / GEAR_REDUCTION
            rpm = (output_revolutions / dt_seconds) * 60.0
            
            self.speed_pub.publish(rpm)
            rate.sleep()

if __name__ == '__main__':
    node = EncoderNode()
    node.run()
