#!/usr/bin/env python3

from functools import partial
import rospy
import RPi.GPIO as GPIO
from std_msgs.msg import Int32
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from drivers.srv import JointTrajectoryServ, JointTrajectoryServResponse
from time import sleep

QTD_JOINTS = 1
SERVICE_SUCCESS = Int32(0)
SERVICE_ERROR = Int32(1)
FREQ_PWM = 50

def angle_to_duty_cycle(angle):
    '''
    -> Regrinha de 3

    Entrada: Ângulo entre -90 e 90
    Saída: Valores entre 0 e 100
    '''
    diff = - 1
    _min = 2.5 + diff
    _max = 12.5 + diff
    angle = -min(90, max(-90, angle))
    return (angle + 90) * (_max - _min) / 180 + _min

current_angle = 0
def change_pwm(pwm_gpio, angle):
    global current_angle
    pwm_gpio.ChangeDutyCycle(angle_to_duty_cycle(angle))
    sleep(0.01 * abs(angle - current_angle))
    sleep(0.5)
    pwm_gpio.ChangeDutyCycle(0)
    current_angle = angle

def callback_servo_service(pwm, msg):
    msg = msg.Content
    res = JointTrajectoryServResponse()
    res.Exit = SERVICE_ERROR
    if len(msg.joint_names) != QTD_JOINTS: return res
    for point in msg.points:
        if len(point.positions) != QTD_JOINTS: return res
        angle = point.positions[0]
        change_pwm(pwm, angle)

    # Ended the Service successfully
    res.Exit = SERVICE_SUCCESS
    return res

class ServoMotorNode:
    def __init__(self):
        rospy.init_node('servo_motor_node')
        # TODO: REMOVE THIS AND SET THE PIN FROM ROSLAUNCH
        rospy.set_param("~SERVO_PIN", "6")
        self.servo_pin = int(rospy.get_param("~SERVO_PIN"))

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.servo_pin, GPIO.OUT)
        pwm = GPIO.PWM(self.servo_pin, FREQ_PWM)
        pwm.start(0)
        change_pwm(pwm, 0)

        callback = partial(callback_servo_service, pwm)
        self.service = rospy.Service('/servo_motor/set_camera_angle', JointTrajectoryServ, callback)

if __name__ == "__main__":
    node = ServoMotorNode()
    rospy.on_shutdown(lambda: GPIO.cleanup(node.servo_pin))
    rospy.spin()
