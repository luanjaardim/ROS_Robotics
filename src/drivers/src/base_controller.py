#!/usr/bin/env python3

import rospy
import RPi.GPIO as GPIO
import math
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from enum import Enum
from simple_pid import PID

PULSES_PER_REV = 11
GEAR_REDUCTION = 46
RATE_HZ = 100

class Encoders:
    def __init__(self, encoder_l_CH1=27, encoder_l_CH2=17, encoder_r_CH1=23, encoder_r_CH2=24):
        # Set up GPIO mode and encoder pins
        self.encoder_l_CH1 = encoder_l_CH1
        self.encoder_l_CH2 = encoder_l_CH2
        self.encoder_r_CH1 = encoder_r_CH1
        self.encoder_r_CH2 = encoder_r_CH2

        GPIO.setup(self.encoder_l_CH1, GPIO.IN)
        GPIO.setup(self.encoder_l_CH2, GPIO.IN)
        GPIO.setup(self.encoder_r_CH1, GPIO.IN)
        GPIO.setup(self.encoder_r_CH2, GPIO.IN)

        # Initialize pulse counts
        self.pulses_count_left = 0
        self.pulses_count_right = 0

        # Set up encoder event detection with callbacks
        GPIO.add_event_detect(self.encoder_l_CH1, GPIO.BOTH, callback=self._encoder_l_callback)
        GPIO.add_event_detect(self.encoder_r_CH1, GPIO.BOTH, callback=self._encoder_r_callback)

        # Set up GPIO cleanup on shutdown
        rospy.on_shutdown(GPIO.cleanup)

        # Initialize last_time for computing delta time
        self.last_time = rospy.Time.now()

    def _encoder_l_callback(self, channel):
        if GPIO.input(self.encoder_l_CH1) != GPIO.input(self.encoder_l_CH2):
            self.pulses_count_left += 1
        else:
            self.pulses_count_left -= 1

    def _encoder_r_callback(self, channel):
        if GPIO.input(self.encoder_r_CH1) != GPIO.input(self.encoder_r_CH2):
            self.pulses_count_right += 1
        else:
            self.pulses_count_right -= 1

    def read_and_reset_counts(self):
        """Return current pulse counts and reset them to zero."""
        left = self.pulses_count_left
        right = self.pulses_count_right
        self.pulses_count_left = 0
        self.pulses_count_right = 0
        return left, right

    def compute_rpm(self):
        """
        Computes and returns the RPM for the left and right encoders.
        This method calculates the elapsed time, converts pulse counts to motor
        revolutions, applies gear reduction, and then converts to RPM.
        """
        # Calculate time difference
        current_time = rospy.Time.now()
        dt_seconds = (current_time - self.last_time).to_sec()
        self.last_time = current_time
        if dt_seconds <= 0:
            dt_seconds = 1 / RATE_HZ

        # Read and reset pulse counts
        pulses_left, pulses_right = self.read_and_reset_counts()

        # Convert pulses to motor revolutions
        motor_revolutions_left = pulses_left / PULSES_PER_REV
        motor_revolutions_right = pulses_right / PULSES_PER_REV

        # Account for gear reduction
        output_revolutions_left = motor_revolutions_left / GEAR_REDUCTION
        output_revolutions_right = motor_revolutions_right / GEAR_REDUCTION

        # Convert revolutions per second to RPM
        rpm_left = (output_revolutions_left / dt_seconds) * 60.0
        rpm_right = (output_revolutions_right / dt_seconds) * 60.0

        return rpm_left, rpm_right

class MotorsEnum(Enum):
    LEFT = 'L'
    RIGHT = 'R'

class Motors:
    def __init__(self):
        self.MOTOR_L_IN1 = 13
        self.MOTOR_L_IN2 = 19
        self.MOTOR_R_IN1 = 18
        self.MOTOR_R_IN2 = 12

        GPIO.setup(self.MOTOR_L_IN1, GPIO.OUT)
        GPIO.setup(self.MOTOR_L_IN2, GPIO.OUT)
        GPIO.setup(self.MOTOR_R_IN1, GPIO.OUT)
        GPIO.setup(self.MOTOR_R_IN2, GPIO.OUT)

        self.motor_l_in1 = GPIO.PWM(self.MOTOR_L_IN1, 1000)
        self.motor_l_in2 = GPIO.PWM(self.MOTOR_L_IN2, 1000)
        self.motor_r_in1 = GPIO.PWM(self.MOTOR_R_IN1, 1000)
        self.motor_r_in2 = GPIO.PWM(self.MOTOR_R_IN2, 1000)

        self.motor_l_in1.ChangeDutyCycle(0)
        self.motor_l_in2.ChangeDutyCycle(0)
        self.motor_r_in1.ChangeDutyCycle(0)
        self.motor_r_in2.ChangeDutyCycle(0)


    def run(self, motor: MotorsEnum, power):

        if motor == MotorsEnum.LEFT:
            if (power > 0):
                self.motor_l_in1.ChangeDutyCycle(abs(power))
                self.motor_l_in2.ChangeDutyCycle(0)
            else:
                self.motor_l_in1.ChangeDutyCycle(0)
                self.motor_l_in2.ChangeDutyCycle(abs(power))
        else:
            if (power > 0):
                self.motor_r_in1.ChangeDutyCycle(abs(power))
                self.motor_r_in2.ChangeDutyCycle(0)
            else:
                self.motor_r_in1.ChangeDutyCycle(0)
                self.motor_r_in2.ChangeDutyCycle(abs(power))


class BaseController:
    def __init__(self):
        rospy.init_node('base_controller_node')
        self.speeds_pub = rospy.Publisher('wheel_speeds', Float32MultiArray, queue_size=1)
        rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_callback)

        # Instantiate Encoders and Motors classes
        GPIO.setmode(GPIO.BCM)
        self.encoders = Encoders()
        self.motors = Motors()

        # Cinematic model parameters
        self.wheel_radius = 0.03
        self.wheel_base = 0.12

        # Disered speeds
        self.desired_speed_left = 0
        self.desired_speed_right = 0

        # PID controllers
        self.pid_left = PID(1, 0.1, 0.05, setpoint=0)
        self.pid_left.output_limits = (-100, 100)

        self.pid_right = PID(1, 0.1, 0.05, setpoint=0)
        self.pid_right.output_limits = (-100, 100)

    def cmd_vel_callback(self, msg):
        v = msg.linear.x
        w = msg.angular.z

        v_right = (2 * v + w * self.wheel_base) / (2 * self.wheel_radius)
        v_left = (2 * v - w * self.wheel_base) / (2 * self.wheel_radius)

        self.desired_speed_right = v_right * 60 / (2 * math.pi)
        self.desired_speed_left = v_left * 60 / (2 * math.pi)

        self.pid_left.setpoint = self.desired_speed_left
        self.pid_right.setpoint = self.desired_speed_right


    def run(self):
        rate = rospy.Rate(RATE_HZ)
        while not rospy.is_shutdown():

            # Publish encoders speeds
            rpm_left, rpm_right = self.encoders.compute_rpm()
            
            data = Float32MultiArray()
            data.data = [rpm_left, rpm_right]
            self.speeds_pub.publish(data)

            # # Test motors drivers
            # self.motors.run(MotorsEnum.LEFT, 100)
            # self.motors.run(MotorsEnum.RIGHT, 100)

            pwm_left = self.pid_left(rpm_left)
            pwm_right = self.pid_right(rpm_right)

            self.motors.run(MotorsEnum.LEFT, pwm_left)
            self.motors.run(MotorsEnum.RIGHT, pwm_right)

            rate.sleep()


if __name__ == '__main__':
    try:
        base_controller = BaseController()
        base_controller.run()
    except rospy.ROSInterruptException:
        pass