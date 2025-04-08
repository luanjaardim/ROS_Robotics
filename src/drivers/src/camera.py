#!/usr/bin/env python3

from functools import partial
import rospy
import RPi.GPIO as GPIO
from sensor_msgs.msg import Image
from drivers.srv import CameraServ, CameraServResponse
from time import sleep
from cv_bridge import CvBridge
import cv2

def callback_camera_service(msg):
    res = CameraServResponse()

    # open camera
    cap = cv2.VideoCapture ("/dev/video0", cv2.CAP_V4L)

    # set dimensions
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560//4)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1440//4)

    # take frame
    ret, image = cap.read()
    #image = cv2.rotate(frame, cv2.ROTATE_180)
    cv2.imwrite("photo.png", image)
    # release camera
    cap.release()

    if not ret:
       rospy.logerr("Error converting OpenCV image to ROS message: %s", e)

    # Convert the frame to a ROS sensor_msgs/Image
    try:
        bridge = CvBridge()
        res.img = bridge.cv2_to_imgmsg(image, encoding="rgb8")
        return res
    except Exception as e:
        rospy.logerr("Error converting OpenCV image to ROS message: %s", e)

class CameraNode:
    def __init__(self):
        rospy.init_node('camera_node')
        self.service = rospy.Service('/camera_get_image', CameraServ, callback_camera_service)

if __name__ == "__main__":
    node = CameraNode()
    rospy.spin()
