#!/usr/bin/env python3
# Copyright (c) 2023 Lee Perry

import importlib.util
import os
import subprocess

import rclpy
import rclpy.node
import std_msgs.msg
try: import ignition.msgs
except ImportError: pass

from neuros.hooks import Hooks
from neuros.config import NodeConfig

class Node:

    @classmethod
    def _make_qos(cls, reliable):
        return rclpy.qos.QoSProfile(
            reliability = rclpy.qos.ReliabilityPolicy.RELIABLE
                          if reliable else
                          rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability  = rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            history     = rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth       = 1)

    class _RosNode(rclpy.node.Node):

        def __init__(self, name):
            super().__init__(name)

    def __init__(self, config):
        self._node = Node._RosNode(config.name)
        self._config = config
        self._plugins = set()
        self._user_data = None

        # this must always be the last line as callbacks are invoked here!
        self._hooks = Hooks(self, config)

    def get_parameter(self, name):
        return self._config.raw_data["node"].get("parameters", {}).get(name)

    def get_ros_node(self):
        return self._node

    def set_user_data(self, data):
        self._user_data = data

    def get_user_data(self):
        return self._user_data

    def get_ros_topic_list(self):
        return [str(topic, "utf-8") for topic in
                subprocess.check_output("ros2 topic list", shell=True).split()]

    def get_ros_topic_info(self, topic):
        return str(subprocess.check_output(
            f"ros2 topic info {topic}", shell=True), "utf-8")

    def get_ign_topic_list(self):
        return [str(topic, "utf-8") for topic in
                subprocess.check_output("ign topic -l", shell=True).split()]

    def get_ign_topic_info(self, topic):
        return str(subprocess.check_output(
            f"ign topic -i -t {topic}", shell=True), "utf-8")

    def load_plugin(self, directory, filename):
        module_name = os.path.splitext(filename)[0]
        full_path = os.path.join(directory, filename)
        if full_path not in self._plugins:
            spec = importlib.util.spec_from_file_location(module_name, full_path)
            impl = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(impl)
        self._plugins.add(full_path)

    def find_type_by_name(self, name):
        try:
            obj = eval(name)
            if isinstance(obj, type):
                return obj
        except Exception:
            pass
        raise Exception(f"Invalid plugin type: {name}") from None

    def make_packet(self, output_name=None):
        return (std_msgs.msg.Empty()
                if output_name is None else
                self._hooks.outputs[output_name].create_packet())

    def make_external_input(self, topic, packet_type, callback):
        return self._node.create_subscription(
            packet_type, topic, callback, Node._make_qos(True))

    def make_internal_input(self, connection, packet_type, callback):
        topic = "/".join([connection.source_node,
                          connection.source_output,
                          connection.destination_node])
        return (self._node.create_subscription(
                    packet_type,
                    f"{topic}/data",
                    callback,
                    Node._make_qos(connection.is_reliable)),
                self._node.create_publisher(
                    std_msgs.msg.Empty,
                    f"{topic}/ack",
                    Node._make_qos(True))
                    if connection.is_reliable else None,
                self._node.create_publisher(
                    std_msgs.msg.Empty,
                    f"{topic}/reg",
                    Node._make_qos(True)))

    def make_external_output(self, topic, packet_type):
        return self._node.create_publisher(
            packet_type, topic, Node._make_qos(True))

    def make_internal_output(self, connection, packet_type, ack_cb, reg_cb):
        topic = "/".join([connection.source_node,
                          connection.source_output,
                          connection.destination_node])
        return (self._node.create_publisher(
                    packet_type,
                    f"{topic}/data",
                    Node._make_qos(connection.is_reliable)),
                self._node.create_subscription(
                    std_msgs.msg.Empty,
                    f"{topic}/ack",
                    ack_cb,
                    Node._make_qos(True)),
                self._node.create_subscription(
                    std_msgs.msg.Empty,
                    f"{topic}/reg",
                    reg_cb,
                    Node._make_qos(True)))

    def make_timer(self, seconds, callback):
        return self._node.create_timer(seconds, callback)

    # TODO
    # Add shutdown() method which sends out a message to all nodes,
    # indicating that the experiment is over

def main():
    # Fix Ctrl-C with https://stackoverflow.com/questions/48256374/python-class-instance-member-variable-isnt-being-updated-inside-thread
    rclpy.init()
    node = Node(NodeConfig.from_standard_node_dir())
    rclpy.spin(node.get_ros_node())
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
