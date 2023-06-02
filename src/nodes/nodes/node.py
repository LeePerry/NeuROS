import rclpy
from rclpy.node import Node as RosNode

from nodes.hooks import initialise_hooks
from nodes.node_config import NodeConfig
from nodes.plugin import plugin_import
from nodes.receiver import Receiver
from nodes.sender import Sender

class Node(RosNode):

    def __init__(self, config):
        super().__init__(config.get_name())
        plugin_import(NodeConfig.standard_project_dir, config.get_source())
        self._config = config
        self._senders = Sender.for_node(self)
        for hook in initialise_hooks:
            hook(self)
        self._receivers = Receiver.for_node(self)

    def get_outgoing(self, name):
        return self._senders[name]

    def get_incoming(self, name):
        return self._receivers[name]

    def get_config(self):
        return self._config

def main():
    rclpy.init()
    rclpy.spin(Node(NodeConfig.from_standard()))
    rclpy.shutdown()

if __name__ == '__main__':
    main()
