from threading import Semaphore

from std_msgs.msg import String as String

from neuros.quality_of_service import standard_quality

class AckServer:

    class Session:

        def __init__(self, name):
            self.name = name
            self.no_acks = 0
            self.semaphore = Semaphore()

    def __init__(self, node, topic, members, max_permitted_no_ack=0):
        self._all_members = { name : AckServer.Session(name) for name in members }
        self._subscriber = node.create_subscription(
            String,
            topic,
            self._register,
            standard_quality())
        self._max_permitted_no_ack = max_permitted_no_ack

    def _register(self, name):
        self._all_members[name.data].semaphore.release()

    def wait_for_all(self):
        for _, member in self._all_members.items():
            member.no_acks += 1
            if member.no_acks > self._max_permitted_no_ack:
                member.semaphore.acquire()
                member.no_acks = 0

class AckClient:

    def __init__(self, node, topic):
        self._node = node
        self._timer = None
        self._registration_publisher = self._node.create_publisher(
            String,
            topic,
            standard_quality())

    def send(self):
        packet = String()
        packet.data = self._node.get_name()
        self._registration_publisher.publish(packet)

    def start_timer(self, interval):
        self._timer = self._node.create_timer(interval, self.send)

    def stop_timer(self):
        self._timer.cancel()
