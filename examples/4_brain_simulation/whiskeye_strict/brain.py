# Copyright (c) 2023 Lee Perry

from neuros.hooks import neuros_initialise, neuros_function

from cv_bridge import CvBridge, CvBridgeError
import cv2

@neuros_initialise(outputs=["neck_pos_cmd", "robot_vel_cmd"])
def initialise(node):
    node.set_user_data(CvBridge())
    neck_pos = node.make_packet("neck_pos_cmd")
    neck_pos.data = [ 60.0, 60.0, 0.0]
    robot_vel = node.make_packet("robot_vel_cmd")
    robot_vel.data = [ 0.0, 0.0, 1.0 ]
    return neck_pos, robot_vel

@neuros_function(inputs="imu")
def receive_imu(node, imu):
    node.get_ros_node().get_logger().info("Received imu")

def save_image(bridge, camera, image):
    cv2_img = bridge.imgmsg_to_cv2(image, "bgr8")
    time = image.header.stamp
    cv2.imwrite(f"images/{camera}/{time.sec}_{time.nanosec}.jpeg", cv2_img)

@neuros_function(inputs="cam0")
def receive_cam0_image(node, image):
    node.get_ros_node().get_logger().info("Received cam0 image")
    #save_image(node.get_user_data(), "cam0", image)

@neuros_function(inputs="cam1")
def receive_cam1_image(node, image):
    node.get_ros_node().get_logger().info("Received cam1 image")
    #save_image(node.get_user_data(), "cam1", image)

@neuros_function(inputs="cam2")
def receive_cam2_image(node, image):
    node.get_ros_node().get_logger().info("Received cam2 image")
    #save_image(node.get_user_data(), "cam2", image)
