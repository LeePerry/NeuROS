# Copyright (c) 2023 Lee Perry

from neuros.hooks import neuros_initialise, neuros_function

from cv_bridge import CvBridge
import cv2

def timestamp_to_seconds(ts):
    return ts.sec + (ts.nanosec / 1_000_000_000)

class Logger:

    def __init__(self, logger):
        self._image_count = 0
        self._gt_count = 0
        self._logger = logger
        self.cv_bridge = CvBridge()
        self.csv_path = "images/ground_truth.csv"
        self.csv = open(self.csv_path, "w")
        self.csv.write("Timestamp,X,Y,Z,W\n")

    def save_image(self, image):
        seconds = timestamp_to_seconds(image.header.stamp)
        cv2_img = self.cv_bridge.imgmsg_to_cv2(image, "bgr8")
        image_path = f"images/{seconds}.jpeg"
        cv2.imwrite(image_path, cv2_img)
        self._image_count += 1
        self._logger(f"Image [{self._image_count}]: {image_path}")

    def save_ground_truth(self, ground_truth):
        seconds = timestamp_to_seconds(ground_truth.header.stamp)
        rot = ground_truth.pose.orientation
        self.csv.write(f"{seconds},{rot.x},{rot.y},{rot.z},{rot.w}\n")
        self._gt_count += 1
        self._logger(f"Ground truth [{self._gt_count}]: {self.csv_path}")

@neuros_initialise()
def initialise(node):
    logger = node.get_ros_node().get_logger().info
    node.set_user_data(Logger(logger))

@neuros_function(inputs="ground_truth")
def receive_ground_truth(node, ground_truth):
    logger = node.get_user_data()
    logger.save_ground_truth(ground_truth)

@neuros_function(inputs="camera")
def receive_camera_image(node, image):
    logger = node.get_user_data()
    logger.save_image(image)
