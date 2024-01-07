# Copyright (c) 2023 Lee Perry

"""
This module is expected to be imported by user plugins. It provides a number of
decorators which can be used to invoke user hooks when specific conditions
are met. These conditions will typically include: Node initialisation,
receiving new data from source nodes, destination nodes becoming ready to
receive more data or expired timers.
"""

from neuros.config import FileSystem
from neuros.input import Input, Timer
from neuros.output import Output

class All:

    """
    This class is used as an input modifier, in order to indicate that the
    node should wait until ALL of the source nodes have provided their data.
    This is in contrast to not specifying this modifier, whereby the input
    would be satisfied when ANY source node provided their data. In the case
    it is specified, then the variable recieved by the user hook will be a
    list, where each element contains data from one of the source nodes. See
    the included 2_synchronisation/voting example.
    """

    def __init__(self, value):
        """
        Initialises a new instance of the All modified class.

        Parameters:
            value (str): The name of the input to modify.
        """
        self.value = value

class Optional:

    """
    This class is used as input/output modifier. In the case of an input, this
    indiates that the hook can be invoked even when the input has not yet
    been received. For such an invocation, the variable passed to the user
    hook will be equal to None. In the case of an output, it indicates that
    the hook may or may not produce the output. Either it can return None
    in it's place, or execute no return statement at all.
    """

    def __init__(self, value):
        """
        Initialises a new instance of the Any modifier class.

        Parameters:
            value (str): The name of the input of output to modify.
        """
        self.value = value

class _Function:

    """
    An internal type which used to represent a user defined hook. It
    consists of a hook method, a number of inputs and outputs in addition
    to their corresponding state.

    Attributes:
        is_waiting (boolean): False if all non-Optional inputs have been
                              received, True otherwise.
        is_blocked (boolean): False if all destination nodes have registered and
                              no output has exceeded it's discard limit, True
                              otherwise. In effect, this indicates that more
                              data can be sent destination.
        is_retired (boolean): True if this hook should only be called once
                              and it has been. This is used for node
                              initialisation.
    """

    class Arg:

        """
        A representation of a function argument.

        Attributes:
            name (str): The name of the corresponding input.
            data: The latest received input data, or None if no data has
                  been received.
            require_all (boolean): Indicates if this input was modified with
                                   the All modified. If so, one packet from
                                   each source node should be collected
                                   together into a list.
            is_optional (boolean): Indicates if this input was modified with
                                   the Optional modifier. If so, it will not
                                   cause is_waiting to be True.
            is_waiting (boolean): Indicates if this argument is still waiting
                                  to receive data from source nodes.
            input: The NeuROS communication resources (see input.py).
        """

        def __init__(self, name, node, all_inputs):
            """
            Initialises a new instance of this class.

            Parameters:
                name (str): The name of the corresponding input.
                node (Node): The corresponding NeuROS node.
                all_inputs (dict): A dictionary of all input names to the
                                   corresponding NeuROS communication
                                   resources.
            """
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
            """
            Checks to see if this argument has been fully satisfied and updates
            the is_waiting member accordingly.
            """
            if self.require_all:
                self.data[source_node] = packet
                self.is_waiting = not (self.is_optional or
                    (set(self.data.keys()) == set(self.input.all_sources())))
            else:
                self.data = packet
                self.is_waiting = False

        def value(self):
            """
            Getter for the value associate with this argument.

            Returns:
                In the case of a single input, this is equivalent to the
                received packet (or None, if no packet has been received).
                In the case of an All input, this will be equal to a list,
                where each element of the list contains the received packet
                from one of the source nodes.
            """
            if self.require_all: return self.data.values()
            else:                return self.data

        def clear(self):
            """
            Resets the state of this argument ready to receive the next round
            of inputs.
            """
            if self.require_all: self.data.clear()
            else:                self.data = None
            if not self.is_optional:
                self.is_waiting = True

    class ReturnValue:

        """
        A representation of a function return value.

        Attributes:
            name (str): The name of the corresponding output.
            node (Node): The corresponding NeuROS node.
            is_optional (boolean): Indicates if this output was modified with
                                   the Optional modifier. If so, the hook
                                   function doesn't necessarily need to provide
                                   a value for it.
            is_blocked (boolean): Indicates if the output has exceeded it's
                                  discard limit and needs to wait for
                                  destination nodes to become ready again.
            output: The NeuROS communication resources (see output.py).
        """

        def __init__(self, name, node, all_outputs):
            """
            Initialises a new instance of this class.

            Parameters:
                name (str): The name of the corresponding output.
                all_outputs (dict): A dictionary of all output names to the
                                    corresponding NeuROS communication
                                    resources.
            """
            self.name = name
            self.is_optional = False
            if isinstance(self.name, All):
                raise Exception("Outputs cannot be specified with All")
            elif isinstance(self.name, Optional):
                self.is_optional = True
                self.name = self.name.value
            self.output = all_outputs[self.name]
            self.is_blocked = self.output.is_blocked

        def check_blocked(self):
            """
            Check to see if this return value is blocked and updated the
            is_blocked member member accordingly.
            """
            self.output.check_blocked()
            self.is_blocked = self.output.is_blocked

    def __init__(self, node, func, named_inputs=[], named_outputs=[],
                 all_inputs={}, all_outputs={}, timer=None, fire_once=False):
        """
            Initialises a new instance of this class.

            Parameters:
                node (Node): The corresponding NeuROS node.
                func (function): The user defined hook to be invoked.
                named_inputs (str or list): The list of inputs that are
                                            required by func.
                named_outputs (str or list): The list of outputs that are
                                             generated by func.
                all_inputs (dict): A dictionary of all input names to NeuROS
                                   Input associated with this node.
                all_outputs (str or list): The list of all output names to
                                           NeuROS Output associated with this
                                           node.
                timer (Timer): An instace of a NeuROS Timer.
                fire_once (boolean): Indicates if this hook should be
                                     retired immediately after firing the first
                                     time.
        """

        if not isinstance(named_inputs, list):
            named_inputs = [named_inputs]
        if not isinstance(named_outputs, list):
            named_outputs = [named_outputs]

        self._node = node
        self._func = func
        self._args = [_Function.Arg(n, node, all_inputs)
                      for n in named_inputs]

        self._returns = [_Function.ReturnValue(n, node, all_outputs)
                         for n in named_outputs]
        self._timer = timer
        self.is_retired = False
        self.is_waiting = False
        self.check_waiting()
        self.is_blocked = any(r.is_blocked for r in self._returns)
        self.check_blocked()
        self._fire_once = fire_once

    def input(self, name, source_node, packet):
        """
        Input has been received from a source node.

        Parameters:
            name (str): The name of the received input.
            source_node (str): The name of the node that sent the data.
            packet: The data received.
        """
        for a in self._args:
            if a.name == name:
                a.check_waiting(source_node, packet)
        self.check_waiting()

    def check_waiting(self):
        """
        Checks to see if either an associated timer or any argument is
        waiting and updates the is_waiting member accordingly.
        """
        if self._timer and self._timer.is_waiting:
            self.is_waiting = True
        else:
            self.is_waiting = any(a.is_waiting for a in self._args)

    def check_blocked(self):
        """
        Checks to see if any return value is blocked and updates the is_blocked
        member accordingly.
        """
        for r in self._returns:
            r.check_blocked()
        self.is_blocked = any(r.is_blocked for r in self._returns)

    def fire(self):
        """
        Invoke the user-defined hook, passing all received inputs and
        validating then sending any outputs. Finally, clears any timer and
        resets inputs ready for the next receipt.
        """
        inputs = [a.value() for a in self._args]
        outputs = self._func(self._node, *inputs)
        if self._fire_once:
            self.is_retired = True

        if outputs is None:
            if any(not r.is_optional for r in self._returns):
                raise Exception(f"{self._func.__name__}: " +
                                "Non-optional outputs not provided")
        else:
            if isinstance(outputs, tuple):
                outputs = list(outputs)
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

    """
    This class represents the complete set of user defined hooks that are
    assocuated with a particular node.
    """

    _on_init = []
    _on_tick = []
    _on_idle = []

    @classmethod
    def add(cls, hooks, arg):
        """
        This method returns a decorator which can add a new hook to a specified
        list.

        Parameters:
            hooks (list): The list to add the new hook to.
            arg (list): The set of hook configuration parameters.

        Returns:
            A decorator capable of adding a function to a list.
        """
        def process_args(func):
            hooks.append((arg, func))
            def process_func(node):
                return func(node)
            return process_func
        return process_args

    def __init__(self, node, config):
        """
        Initialises a new instance of this class.

        Parameters:
            node (Node): The node to associated these hooks with.
            config (NodeConfig): The node configuration including all specified
                                 inputs and outputs.
        """
        node.load_plugin(FileSystem.standard_project_dir, config.plugin)
        self.inputs = Input.for_node(node, config, self._input_cb)
        self.outputs = Output.for_node(node, config, self._reg_cb, self._ack_cb)
        self._registration_complete = all(o.is_registered
                                          for o in self.outputs.values())

        # create and add all initialisation hooks
        self._hooks = [_Function(node,
                                 func,
                                 named_outputs=outputs,
                                 all_outputs=self.outputs,
                                 fire_once=True)
                                 for outputs, func in Hooks._on_init]

        # create and add all timed hooks
        self._hooks += [_Function(node,
                                  func,
                                  timer=Timer(node, inout[0], self._tick),
                                  named_outputs=inout[1],
                                  all_outputs=self.outputs)
                                  for inout, func in Hooks._on_tick]

        # create and add all input/output function hooks
        self._hooks += [_Function(node,
                                  func,
                                  named_inputs=inout[0],
                                  named_outputs=inout[1],
                                  all_inputs=self.inputs,
                                  all_outputs=self.outputs)
                                  for inout, func in Hooks._on_idle]

        self._loop_sub, self._loop_pub = node.make_loop_back(self._fire_hooks)
        self._loop_packet = node.make_packet()
        self._queue_fire_hooks()

    def _reg_cb(self):
        """
        A destination node has registered itself with this node.
        """
        self._registration_complete = all(o.is_registered
                                          for o in self.outputs.values())
        self._fire_hooks()

    def _input_cb(self, name, source_node, packet):
        """
        A source node has delivered some data to this node.

        Parameters:
            name (str): The name of the input received.
            source_node (str): The name of the node that sent the data.
            packet: The data received from the source node.
        """
        for hook in self._hooks:
            hook.input(name, source_node, packet)
        self._fire_hooks()

    def _ack_cb(self):
        """
        An acknowledgment has been received from a destination node. This
        indicates that the destination node has beguin work on the data this
        node sent to them, and that some more data can now be queued.
        """
        for hook in self._hooks:
            hook.check_blocked()
        self._fire_hooks()

    def _tick(self):
        """
        A timer has expired. Check to see if this hook is ready to be fired.
        """
        for hook in self._hooks:
            hook.check_waiting()
        self._fire_hooks()

    def _queue_fire_hooks(self):
        """
        Add a packet to our own queue to indicate that we should fire hooks
        again. This is useful to ensure that other packets have a chance to
        be processed beforehand.
        """
        self._loop_pub.publish(self._loop_packet)

    def _fire_hooks(self, _=None):
        """
        Fire any hooks that are not waiting or blocked.
        """
        if self._registration_complete:
            fired_hooks = []

            # fire any hooks that are ready
            for hook in self._hooks:
                hook.check_blocked()
                hook.check_waiting()
                if not (hook.is_waiting or hook.is_blocked):
                    hook.fire()
                    fired_hooks.append(hook)

            # move any fired hooks to the end of the queue,
            # ensuring that no hook is starved by another.
            # (and remove any retired hooks)
            for hook in fired_hooks:
                self._hooks.remove(hook)
                if not hook.is_retired:
                    self._hooks.append(hook)

            # if any are ready, queue another fire_hooks.
            # this is preferable to refiring as it allows
            # new inputs to be processed.
            if not all((h.is_waiting or h.is_blocked) for h in self._hooks):
                self._queue_fire_hooks()

def neuros_initialise(**params):
    """
    A decorator for user defined plugins to indicate that a method should be
    invoked when the node is initialised and not waiting for destination nodes
    to become ready to receive the produced outputs.

    Parameters:
        outputs (str or list): Either a single string, or a list of strings,
                               where each string identifies an output that this
                               method can produce.
    """
    return Hooks.add(Hooks._on_init,
                     params.get("outputs", []))

def neuros_tick(**params):
    """
    A decorator for user defined plugins to indicate that a method should be
    invoked periodically, with a specified interval, when not waiting for
    destination nodes to become ready to receive the produced outputs.

    seconds (double): The length of the interval in seconds. Note that this
                      variable has double precision for specifying sub-second
                      intervals.
    outputs (str or list): Either a single string, or a list of strings,
                           where each string identifies an output that this
                           method can produce. Note that any of these strings
                           can additionally be wrapped in the Optional
                           modifier.
    """
    return Hooks.add(Hooks._on_tick,
                     (params["seconds"], params.get("outputs", [])))

def neuros_function(**params):
    """
    A decorator for user defined plugins to indicate that a method should be
    invoked if all inputs are satisfied and the node is not waiting for
    destination nodes to become ready to receive the produced outputs.

    inputs (str or list): Either a single string, or a list of strings, where
                          each string identifies an input that this method
                          should receive. Note that any of these strings
                          can additionally be wrapped in the All or Optional
                          modifiers.
    outputs (str or list): Either a single string, or a list of strings,
                           where each string identifies an output that this
                           method can produce. Note that any of these strings
                           can additionally be wrapped in the Optional
                           modifier.
    """
    return Hooks.add(Hooks._on_idle,
                     (params.get("inputs", []), params.get("outputs", [])))
