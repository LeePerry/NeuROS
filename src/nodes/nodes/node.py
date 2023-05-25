import rclpy
from rclpy.node import Node as RosNode

from nodes.node_config import NodeConfig
from nodes.decorators import initialise_hooks, receive_hooks
from nodes.plugin import plugin_import, plugin_message_type

class Node(RosNode):

    class Sender:

        @classmethod
        def for_node(cls, node):
            senders = {}
            for connection in node._config.get_connections():
                if (node.get_name() == connection.get_sender()):
                    senders[connection.get_name()] = cls(node, connection)
            return senders

        def __init__(self, node, connection):
            self._node = node
            self._topic = f"{connection.get_receiver()}/{connection.get_name()}"
            self._message_type = plugin_message_type(connection.get_message_type_name())
            self._publisher = node.create_publisher(self._message_type, self._topic, 10)

        def send(self, output):
            # todo this should block until all recipients have announced they're ready
            #self._node.get_logger().info(f"sending {output.data} to {self._topic}")
            self._publisher.publish(output)

    class Receiver:

        @classmethod
        def for_node(cls, node):
            receivers = {}
            for connection in node._config.get_connections():
                if (node.get_name() == connection.get_receiver()):
                   receivers[connection.get_name()] = cls(node, connection)
            return receivers

        def __init__(self, node, connection):
            self._node = node
            self._hooks = [h for n, h in receive_hooks if n == connection.get_name()]
            self._topic = f"{connection.get_receiver()}/{connection.get_name()}"
            self._message_type = plugin_message_type(connection.get_message_type_name())
            self._subscription = node.create_subscription(self._message_type, self._topic, self._receive, 10)

        def _receive(self, input):
            #self._node.get_logger().info(f"received {input.data}")
            for hook in self._hooks:
                hook(self._node, input)

    def __init__(self, config):
        super().__init__(config.get_name())
        plugin_import(NodeConfig.standard_project_dir, config.get_source())
        self._config = config
        self._senders = Node.Sender.for_node(self)
        self._receivers = Node.Receiver.for_node(self)
        for hook in initialise_hooks:
            self.get_logger().info(f"Initialising node: {config.get_name()}")
            hook(self)

    def _make_input_handler(self, *args, **kwargs):
        def input_handler(*args, **kwargs):
            pass
        return input_handler

    def send(self, connection, message):
        self._senders[connection].send(message)

    def get_config(self):
        return self._config

def main():
    rclpy.init()
    rclpy.spin(Node(NodeConfig.from_standard()))
    rclpy.shutdown()

if __name__ == '__main__':
    main()
