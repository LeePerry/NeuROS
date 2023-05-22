# Copyright (c) 2023 Lee Perry

from neuros.hooks import neuros_initialise, neuros_function

from cv_bridge import CvBridge, CvBridgeError
import cv2

import math
import os

def timestamp_to_us(ts):
    return (ts.sec * 1_000_000) + (ts.nanosec / 1_000)

class Brain:

    def __init__(self, logger):
        self._image_count = 0
        self._logger = logger
        self.cv_bridge = CvBridge()
        self.csv = open("images/ground_truth.csv", "w")
        self.csv.write("Timestamp,X,Y,Z,W\n")

    def save_image(self, camera, image):
        ms = timestamp_to_us(image.header.stamp)
        cv2_img = self.cv_bridge.imgmsg_to_cv2(image, "bgr8")
        cv2.imwrite(f"images/{ms}.jpeg", cv2_img)
        self._image_count += 1
        self._logger(f"Image count: {self._image_count}")

    def save_ground_truth(self, ground_truth):
        ms = timestamp_to_us(ground_truth.header.stamp)
        rot = ground_truth.pose.orientation
        self.csv.write(f"{ms},{rot.x},{rot.y},{rot.z},{rot.w}\n")

@neuros_initialise(outputs=["neck_pos_cmd", "robot_vel_cmd"])
def initialise(node):
    logger = node.get_ros_node().get_logger().info
    node.set_user_data(Brain(logger))
    neck_pos = node.make_packet("neck_pos_cmd")
    neck_pos.data = [ 0.0, math.pi / 2, 0.0]
    robot_vel = node.make_packet("robot_vel_cmd")
    rot_speed = float(os.environ["NEUROS_ROTATION_SPEED"])
    robot_vel.data = [ 0.0, 0.0, rot_speed ]
    return neck_pos, robot_vel

@neuros_function(inputs="ground_truth")
def receive_ground_truth(node, ground_truth):
    brain = node.get_user_data()
    brain.save_ground_truth(ground_truth)

@neuros_function(inputs="camera")
def receive_camera_image(node, image):
    brain = node.get_user_data()
    brain.save_image("camera", image)
