#!/usr/bin/env python3
# Copyright (c) 2023 Lee Perry

from threading import Thread

import rclpy
from rclpy.node import Node as RosNode
from rclpy.executors import MultiThreadedExecutor

from neuros.hooks import Hooks
from neuros.node_config import NodeConfig
from neuros.plugin import plugin_import
from neuros.receiver import Receiver
from neuros.sender import Sender
from neuros.timer import Timer

class Node(RosNode):

    def __init__(self, config):
        super().__init__(config.get_name())
        plugin_import(NodeConfig.standard_project_dir, config.get_source())
        self._config = config
        self._senders = Sender.for_node(self)
        self._receivers = Receiver.for_node(self)
        for hook in Hooks.on_initialise:
            hook(self)
        self._timers = Timer.for_node(self)

    def get_outgoing(self, name):
        return self._senders[name]

    def get_incoming(self, name):
        return self._receivers[0][name]

    def get_config(self):
        return self._config

def main():
    rclpy.init()
    node = Node(NodeConfig.from_standard())

    #while rclpy.ok():
    #    rclpy.spin_once(node)

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    spin_thread = Thread(target=executor.spin, args=())
    spin_thread.start()
    spin_thread.join()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
