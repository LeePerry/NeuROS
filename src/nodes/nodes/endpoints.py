import rclpy.qos as qos
from rclpy.qos import QoSProfile

from nodes.hooks import receive_hooks
from nodes.plugin import plugin_message_type

class Sender:

    @classmethod
    def for_node(cls, node):
        senders = {}
        for connection in node.get_config().get_connections():
            if (node.get_name() == connection.get_sender()):
                key = connection.get_name()
                if key in senders:
                    senders[key].add_recipient(connection.get_receiver())
                else:
                    senders[key] = cls(node, connection)
        return senders

    def __init__(self, node, connection):
        self._node = node
        self._topic = f"{connection.get_receiver()}/{connection.get_name()}"
        self._message_type = plugin_message_type(connection.get_message_type_name())
        self._publisher = node.create_publisher(
            self._message_type,
            self._topic + "/data",
            QoSProfile(
                reliability = qos.ReliabilityPolicy.BEST_EFFORT,
                durability  = qos.DurabilityPolicy.TRANSIENT_LOCAL,
                history     = qos.HistoryPolicy.KEEP_LAST,
                depth       = 1))

    def send(self, output):
        # TODO this should block until all recipients have announced they're ready
        self._publisher.publish(output)

class Receiver:

    @classmethod
    def for_node(cls, node):
        receivers = {}
        for connection in node.get_config().get_connections():
            if (node.get_name() == connection.get_receiver()):
                receivers[connection.get_name()] = cls(node, connection)
        return receivers

    def __init__(self, node, connection):
        self._node = node
        self._hooks = [h for n, h in receive_hooks if n == connection.get_name()]
        self._topic = f"{connection.get_receiver()}/{connection.get_name()}"
        self._message_type = plugin_message_type(connection.get_message_type_name())
        self._subscription = node.create_subscription(
            self._message_type,
            self._topic + "/data",
            self._receive,
            QoSProfile(
                reliability = qos.ReliabilityPolicy.BEST_EFFORT,
                durability  = qos.DurabilityPolicy.TRANSIENT_LOCAL,
                history     = qos.HistoryPolicy.KEEP_LAST,
                depth       = 1))

    def _receive(self, input):
        for hook in self._hooks:
            hook(self._node, input)
