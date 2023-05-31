from threading import Semaphore

import rclpy.qos as qos
from rclpy.qos import QoSProfile
from std_msgs.msg import String as String

from nodes.hooks import receive_hooks
from nodes.plugin import plugin_message_type

def standard_quality():
    return QoSProfile(
        reliability = qos.ReliabilityPolicy.BEST_EFFORT,
        durability  = qos.DurabilityPolicy.TRANSIENT_LOCAL,
        history     = qos.HistoryPolicy.KEEP_LAST,
        depth       = 1)

class Sender:

    @classmethod
    def for_node(cls, node):
        senders = {}
        for connection in node.get_config().get_connections():
            if (node.get_name() == connection.get_sender()):
                key = connection.get_name()
                if key in senders:
                    senders[key].add_receiver(connection)
                else:
                    senders[key] = cls(node, connection)
        return senders

    def __init__(self, node, connection):
        self._node = node
        self._is_ready = False
        self._ready_semaphore = Semaphore()
        self._wait_until_ready = set([connection.get_receiver()])
        self._message_type = plugin_message_type(connection.get_message_type_name())
        self._publisher = self._node.create_publisher(
            self._message_type,
            f"{connection.get_sender()}/{connection.get_name()}/data",
            standard_quality())
        self._subscriber = self._node.create_subscription(
            String,
            f"{connection.get_sender()}/{connection.get_name()}/ready",
            self._receiver_ready,
            standard_quality())

    def add_receiver(self, connection):
        message_type = plugin_message_type(connection.get_message_type_name())
        if self._message_type != message_type:
            raise Exception(f"Connection {connection.get_name()} defined with " +
                            f"inconsistent message type! " +
                            f"Expected: {self._message_type}, " +
                            f"Actual: {message_type}")
        self._wait_until_ready.add(connection.get_receiver())

    def _receiver_ready(self, name):
        self._wait_until_ready.discard(name.data)
        if not self._wait_until_ready:
            self._ready_semaphore.release()

    def send(self, data):
        if not self._is_ready:
            self._ready_semaphore.acquire()
            self._is_ready = True
        self._publisher.publish(data)

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
        self._data_subscription = self._node.create_subscription(
            plugin_message_type(connection.get_message_type_name()),
            f"{connection.get_sender()}/{connection.get_name()}/data",
            self._receive,
            standard_quality())
        self._ready_timer = self._node.create_timer(0.5, self._send_ready)
        self._ready_publisher = self._node.create_publisher(
            String,
            f"{connection.get_sender()}/{connection.get_name()}/ready",
            standard_quality())
        self._latest_data = None
        self._fresh = False

    def _send_ready(self):
        ready = String()
        ready.data = self._node.get_name()
        self._ready_publisher.publish(ready)

    def _receive(self, data):
        self._fresh = True
        self._ready_timer.cancel()
        self._latest_data = data
        for hook in self._hooks:
            hook(self._node)
            self._fresh = False

    def is_fresh(self):
        return self._fresh

    def latest(self):
        return self._latest_data
