# Copyright (c) 2023 Lee Perry

from itertools import cycle

from neuros.config import FileSystem
from neuros.input import Input, Timer
from neuros.output import Output

class All:

    def __init__(self, value):
        self.value = value

class Optional:

    def __init__(self, value):
        self.value = value

class _Function:

    class Arg:

        def __init__(self, name, all_inputs):
            self.name = name
            self.data = None
            self.require_all = False
            self.is_optional = False
            self.is_waiting = True
            while isinstance(self.name, (All, Optional)):
                if isinstance(self.name, All):
                    self.require_all = True
                    self.data = {}
                elif isinstance(self.name, Optional):
                    self.is_optional = True
                    self.is_waiting = False
                self.name = self.name.value
            self.input = all_inputs[self.name]

        def check_waiting(self, source_node, packet):
            if self.require_all:
                self.data[source_node] = packet
                self.is_waiting = not (self.is_optional or
                    (set(self.data.keys()) == set(self.input.all_sources())))
            else:
                self.data = packet
                self.is_waiting = False

        def value(self):
            if self.require_all:
                return self.data.values()
            else:
                return self.data

        def clear(self):
            if self.require_all:
                self.data.clear()
            else:
                self.data = None
            if not self.is_optional:
                self.is_waiting = True

    class ReturnValue:

        def __init__(self, name, all_outputs):
            self.name = name
            self.is_optional = False
            if isinstance(self.name, All):
                raise Exception("Outputs cannot be specified with All")
            elif isinstance(self.name, Optional):
                self.is_optional = True
                self.name = self.name.value
            self.output = all_outputs[self.name]
            self.is_blocked = False

        def check_blocked(self):
            self.is_blocked = self.output.is_blocked

    def __init__(self, node, func, named_inputs=[], named_outputs=[],
                 all_inputs={}, all_outputs={}, timer=None, fire_once=False):

        if not isinstance(named_inputs, list):
            named_inputs = [named_inputs]
        if not isinstance(named_outputs, list):
            named_outputs = [named_outputs]

        self._node = node
        self._func = func
        self._args = [_Function.Arg(n, all_inputs)
                      for n in named_inputs]
        self._returns = [_Function.ReturnValue(n, all_outputs)
                         for n in named_outputs]
        self._timer = timer
        self.is_retired = False
        self.is_waiting = False
        self.check_waiting()
        self.is_blocked = False
        self.check_blocked()
        self._fire_once = fire_once

    def input(self, name, source_node, packet):
        for a in self._args:
            if a.name == name:
                a.check_waiting(source_node, packet)
        self.check_waiting()

    def check_waiting(self):
        if self._timer and self._timer.is_waiting:
            self.is_waiting = True
        else:
            self.is_waiting = any(a.is_waiting for a in self._args)

    def check_blocked(self):
        for r in self._returns:
            r.check_blocked()
        self.is_blocked = any(r.is_blocked for r in self._returns)

    def fire(self):
        inputs = [a.value() for a in self._args]
        outputs = self._func(self._node, *inputs)
        if self._fire_once:
            self.is_retired = True

        if outputs is None:
            if any(not r.is_optional for r in self._returns):
                raise Exception(f"{self._func.__name__}: " +
                                "Non-optional outputs not provided")
        else:
            if not isinstance(outputs, list):
                outputs = [outputs]
            if len(outputs) != len(self._returns):
                raise Exception(f"{self._func.__name__}: " +
                                "Wrong number of outputs provided")
            for o, r in zip(outputs, self._returns):
                if o is not None:
                    r.output.send(o)
                elif not r.is_optional:
                    raise Exception(f"{self._func.__name__}: " +
                                    "Non-optional outputs cannot be None")
            self.check_blocked()

        for a in self._args:
            a.clear()
        if self._timer:
            self._timer.clear()
        self.check_waiting()

class Hooks:

    _on_init = []
    _on_tick = []
    _on_idle = []

    @classmethod
    def add(cls, hooks, arg):
        def process_args(func):
            hooks.append((arg, func))
            def process_func(node):
                return func(node)
            return process_func
        return process_args

    def __init__(self, node, config):
        node.load_plugin(FileSystem.standard_project_dir, config.plugin)
        self.inputs = Input.for_node(node, config, self._input_cb)
        self.outputs = Output.for_node(node, config, self._reg_cb, self._ack_cb)
        self._registration_complete = (len(self.outputs) == 0)

        self._hooks = [_Function(node,
                                 func,
                                 named_outputs=outputs,
                                 all_outputs=self.outputs,
                                 fire_once=True)
                                 for outputs, func in Hooks._on_init]

        self._hooks += [_Function(node,
                                  func,
                                  timer=Timer(node, inout[0], self._tick),
                                  named_outputs=inout[1],
                                  all_outputs=self.outputs)
                                  for inout, func in Hooks._on_tick]

        self._hooks += [_Function(node,
                                  func,
                                  named_inputs=inout[0],
                                  named_outputs=inout[1],
                                  all_inputs=self.inputs,
                                  all_outputs=self.outputs)
                                  for inout, func in Hooks._on_idle]

        self._cycle_hooks = cycle(self._hooks)
        self._fire_hooks()

    def _reg_cb(self):
        self._registration_complete = all(o.is_registered
                                          for o in self.outputs.values())
        self._fire_hooks()

    def _input_cb(self, name, source_node, packet):
        for hook in self._hooks:
            hook.input(name, source_node, packet)
        self._fire_hooks()

    def _ack_cb(self):
        for hook in self._hooks:
            hook.check_blocked()
        self._fire_hooks()

    def _tick(self):
        for hook in self._hooks:
            hook.check_waiting()
        self._fire_hooks()

    def _fire_hooks(self):
        if self._registration_complete:
            not_fired = 0
            for hook in self._cycle_hooks:
                if hook.is_retired or hook.is_waiting or hook.is_blocked:
                    not_fired += 1
                    if not_fired >= len(self._hooks):
                        break
                else:
                    hook.fire()
                    not_fired = 0

def neuros_initialise(**params):
    return Hooks.add(Hooks._on_init,
                     params.get("outputs", []))

def neuros_tick(**params):
    return Hooks.add(Hooks._on_tick,
                     (params["seconds"], params.get("outputs", [])))

def neuros_function(**params):
    return Hooks.add(Hooks._on_idle,
                     (params.get("inputs", []), params.get("outputs", [])))
