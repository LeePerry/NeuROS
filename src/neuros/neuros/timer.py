from neuros.hooks import Hooks

class Timer:

    @classmethod
    def for_node(cls, node):
        return [cls(interval, func, node) for interval, func in Hooks.on_tick]

    def __init__(self, interval, func, node):
        self._func = func
        self._node = node
        self._timer = node.create_timer(interval, self._callback)

    def _callback(self):
        self._func(self._node)
