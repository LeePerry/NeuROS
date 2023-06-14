#!/usr/bin/env python3
# Copyright (c) 2023 Lee Perry

import importlib.util
import os

import rclpy
import rclpy.node
import std_msgs.msg

from neuros.hooks import Hooks
from neuros.config import NodeConfig

class Node(rclpy.node.Node):

    @classmethod
    def _make_neuros_qos(cls, reliable):
        return rclpy.qos.QoSProfile(
            reliability = rclpy.qos.ReliabilityPolicy.RELIABLE
                          if reliable else
                          rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability  = rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            history     = rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth       = 1)

    def __init__(self, config):
        super().__init__(config.name)
        self._config = config
        self._plugins = set()
        self._hooks = Hooks(self, config)

    def get_neuros_parameter(self, name):
        return self._config.raw_data["node"].get(name)

    def load_neuros_plugin(self, directory, filename):
        module_name = os.path.splitext(filename)[0]
        full_path = os.path.join(directory, filename)
        if full_path not in self._plugins:
            spec = importlib.util.spec_from_file_location(module_name, full_path)
            impl = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(impl)
        self._plugins.add(full_path)

    def find_neuros_type_by_name(self, name):
        try:
            obj = eval(name)
            if isinstance(obj, type):
                return obj
        except Exception:
            pass
        raise Exception(f"Invalid plugin type: {name}") from None

    def make_neuros_packet(self, output_name=None):
        return (std_msgs.msg.Empty()
                if output_name is None else
                self._hooks.outputs[output_name].create_packet())

    def make_neuros_input(self, connection, packet_type, callback):
        topic = "/".join([connection.source_node,
                          connection.source_output,
                          connection.destination_node])
        return (self.create_subscription(
                    packet_type,
                    f"{topic}/data",
                    callback,
                    Node._make_neuros_qos(connection.is_reliable)),
                self.create_publisher(
                    std_msgs.msg.Empty,
                    f"{topic}/ack",
                    Node._make_neuros_qos(True))
                    if connection.is_reliable else None,
                self.create_publisher(
                    std_msgs.msg.Empty,
                    f"{topic}/reg",
                    Node._make_neuros_qos(True)))

    def make_neuros_output(self, connection, packet_type, ack_cb, reg_cb):
        topic = "/".join([connection.source_node,
                          connection.source_output,
                          connection.destination_node])
        return (self.create_publisher(
                    packet_type,
                    f"{topic}/data",
                    Node._make_neuros_qos(connection.is_reliable)),
                self.create_subscription(
                    std_msgs.msg.Empty,
                    f"{topic}/ack",
                    ack_cb,
                    Node._make_neuros_qos(True)),
                self.create_subscription(
                    std_msgs.msg.Empty,
                    f"{topic}/reg",
                    reg_cb,
                    Node._make_neuros_qos(True)))

    def make_neuros_timer(self, seconds, callback):
        return self.create_timer(seconds, callback)

def main():
    rclpy.init()
    node = Node(NodeConfig.from_standard_node_dir())
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
