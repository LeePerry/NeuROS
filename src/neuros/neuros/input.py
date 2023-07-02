# Copyright (c) 2023 Lee Perry

from neuros.config import FileSystem

class Subscriber:

    def __init__(self, node, connection, packet_type, input_cb):
        self.source_node = connection.source_node
        self._input_cb = input_cb
        self._data, self._ack, self._reg = node.make_input(
            connection, packet_type, self._input_callback)
        self._reg_timer = node.make_timer(0.5, self._send_reg)
        self._packet = node.make_packet()

    def _send_reg(self):
        self._reg.publish(self._packet)

    def _input_callback(self, packet):
        self._reg_timer.cancel()
        if self._ack is not None:
            self._ack.publish(self._packet)
        self._input_cb(self.source_node, packet)

class Input:

    @classmethod
    def for_node(cls, node, config, input_cb):
        return {i.name : cls(node, i, config, input_cb)
                for i in config.inputs}

    def __init__(self, node, input, config, input_cb):
        self._name = input.name
        if input.plugin:
            node.load_plugin(
                FileSystem.standard_project_dir, input.plugin)
        self._subscribers = [Subscriber(node, c,
            node.find_type_by_name(input.type), self._input_callback)
            for c in config.connections if c.destination_node == config.name]
        self._input_cb = input_cb

    def _input_callback(self, source_node, packet):
        self._input_cb(self._name, source_node, packet)

    def all_sources(self):
        return [s.source_node for s in self._subscribers]

class Timer:

    def __init__(self, node, seconds, callback):
        self.is_waiting = True
        self._callback = callback
        self._timer = node.make_timer(seconds, self._expired)

    def _expired(self):
        self.is_waiting = False
        self._callback()

    def clear(self):
        self.is_waiting = True
