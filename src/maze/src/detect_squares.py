#!/usr/bin/env python3

import cv2
from cv_bridge import CvBridge
import rospy
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from drivers.srv import JointTrajectoryServ, CameraServ, DetectDirServ, DetectDirServResponse
from time import sleep
import numpy as np

DEBUG = True

# HSV thresholds
LOWER_GREEN = np.array([50, 28, 0])
UPPER_GREEN = np.array([83, 134, 150])
LOWER_RED = np.array([0, 100, 100])
UPPER_RED = np.array([255, 255, 255])


def detect_green_circle(mask, output_path):
    """
    Detects a circular green object in the given mask.
    """
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    output = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    detected = False

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 100:
            perimeter = cv2.arcLength(cnt, True)
            if perimeter == 0:
                continue
            circularity = 4 * np.pi * (area / (perimeter * perimeter))
            if 0.7 < circularity < 1.2:
                cv2.drawContours(output, [cnt], -1, (0, 255, 0), 2)
                detected = True
    if DEBUG:
        cv2.imwrite(output_path, output)
    return detected


def detect_red_cross(mask, output_path):
    """
    Detects a red cross shape in the given mask using contour approximation and solidity.
    """
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    output = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    detected = False

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 100:
            epsilon = 0.02 * cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, epsilon, True)
            hull = cv2.convexHull(cnt)
            hull_area = cv2.contourArea(hull)
            if hull_area == 0:
                continue
            solidity = float(area) / hull_area
            if len(approx) >= 5 and solidity < 0.9:
                cv2.drawContours(output, [approx], -1, (0, 0, 255), 2)
                detected = True
    if DEBUG:
        cv2.imwrite(output_path, output)
    return detected


class DetectShapesNode:
    def __init__(self):
        rospy.init_node('detect_shapes_node')
        self.img = None
        self.service = rospy.Service('/detect_shapes', DetectDirServ, self.callback_detect_shapes)

    def call_camera_service(self):
        rospy.wait_for_service('/camera_get_image')
        try:
            camera_service = rospy.ServiceProxy('/camera_get_image', CameraServ)
            response = camera_service()
            self.img = response.img
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

    def call_move_servo_service(self, angle):
        rospy.wait_for_service('/servo_motor/set_camera_angle')
        try:
            move_servo_service = rospy.ServiceProxy('/servo_motor/set_camera_angle', JointTrajectoryServ)
            trajectory = JointTrajectory()
            trajectory.joint_names = ['servo']
            point = JointTrajectoryPoint()
            point.positions = [angle]
            trajectory.points = [point]
            response = move_servo_service(trajectory)
            return response
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
        return None

    def process_side(self, angle, suffix):
        """
        Positions the servo at the given angle, captures the image, and applies shape detection.
        """
        self.call_move_servo_service(angle)
        sleep(1)
        self.call_camera_service()
        sleep(1)
        if self.img is None:
            rospy.logerr("No image received from camera service")
            return None, None

        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(self.img, "rgb8")
        if DEBUG:
            cv2.imwrite(f"/home/robot/robot_ws/camera_images/{suffix}.jpg", img)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask_green = cv2.inRange(hsv, LOWER_GREEN, UPPER_GREEN)
        mask_red = cv2.inRange(hsv, LOWER_RED, UPPER_RED)
        if DEBUG:
            cv2.imwrite(f"mask_green_{suffix}.jpg", mask_green)
            cv2.imwrite(f"mask_red_{suffix}.jpg", mask_red)
        green_circle = detect_green_circle(mask_green, f"/home/robot/robot_ws/camera_images/contours_green_{suffix}.jpg")
        red_cross = detect_red_cross(mask_red, f"/home/robot/robot_ws/camera_images/contours_red_{suffix}.jpg")
        return green_circle, red_cross

    def callback_detect_shapes(self, req):
        res = DetectDirServResponse()
        res.dir = String()
        res.dir.data = "None"

        try:
            # Check the left side
            left_green, left_red = self.process_side(90, "left")
            if left_green:
                res.dir.data = "left"
                return res
            # if left_red:
            #     res.dir.data = "right"
            #     return res

            # Check the right side
            right_green, right_red = self.process_side(-90, "right")
            if right_green:
                res.dir.data = "right"
                return res
            # if right_red:
            #     res.dir.data = "left"
            #     return res

            return res
        finally:
            # Ensure the servo returns to 0 regardless of the execution path
            self.call_move_servo_service(0)


if __name__ == "__main__":
    node = DetectShapesNode()
    rospy.spin()
