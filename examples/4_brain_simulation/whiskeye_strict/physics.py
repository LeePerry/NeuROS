# Copyright (c) 2023 Lee Perry

import os

from neuros.hooks import neuros_initialise, neuros_function, Optional
from neuros.gazebo import Gazebo

STEP_SIZE = 1

@neuros_initialise()
def initialise(node):
    simulator = Gazebo(node, os.environ["NEUROS_WORLD"])
    simulator.spawn_entity(os.environ["NEUROS_ROBOT"])
    simulator.step(STEP_SIZE)
    node.set_user_data(simulator)

@neuros_function(
    inputs  = "neck_pos_cmd",
    outputs = ["_body_neck_pos", "_neck_gmbl_pos", "_gmbl_head_pos"])
def forward_neck_pos_cmd(node, cmd):
    body_neck = node.make_packet("_body_neck_pos")
    body_neck.data = cmd.data[0]
    neck_gmbl = node.make_packet("_neck_gmbl_pos")
    neck_gmbl.data = cmd.data[1]
    gmbl_head = node.make_packet("_gmbl_head_pos")
    gmbl_head.data = cmd.data[2]
    return (body_neck, neck_gmbl, gmbl_head)

@neuros_function(
    inputs  = "robot_vel_cmd",
    outputs = ["_pose_x_vel", "_pose_y_vel", "_pose_theta_vel"])
def forward_robot_vel_cmd(node, cmd):
    pose_x = node.make_packet("_pose_x_vel")
    pose_x.data = cmd.data[0]
    pose_y = node.make_packet("_pose_y_vel")
    pose_y.data = cmd.data[1]
    pose_theta = node.make_packet("_pose_theta_vel")
    pose_theta.data = cmd.data[2]
    return (pose_x, pose_y, pose_theta)

@neuros_function(
    inputs  = [Optional(i) for i in ["_imu", "_cam0", "_cam1", "_cam2"]],
    outputs = [Optional(i) for i in [ "imu",  "cam0",  "cam1",  "cam2"]])
def step(node, *sensor_data):
    simulator = node.get_user_data()
    simulator.step(STEP_SIZE)
    return sensor_data
