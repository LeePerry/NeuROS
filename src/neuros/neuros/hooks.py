class Hooks:

    on_initialise = []
    on_receive = []
    on_tick = []

    @classmethod
    def no_arg(cls, hooks, func):
        hooks.append(func)
        def process_func(*args, **kwargs):
            return func(*args, **kwargs)
        return process_func

    @classmethod
    def single_arg(cls, hooks, arg):
        def process_args(func):
            hooks.append((arg, func))
            def process_func(*args, **kwargs):
                return func(*args, **kwargs)
            return process_func
        return process_args

def neuros_initialise(func):
    return Hooks.no_arg(Hooks.on_initialise, func)

def neuros_receive(connection_name):
    return Hooks.single_arg(Hooks.on_receive, connection_name)

def neuros_tick(seconds):
    return Hooks.single_arg(Hooks.on_tick, seconds)

# TODO consider
#
# neuros_fixed_interval(milliseconds)
#
# neuros_received_any(...)
# neuros_received_all(...)
#
# where ... is a list of connections,
# any=triggers after any of those connections fire,
# all=triggers after all connections have recived new data
#
# neuros_receive_accumulate(connection, count)
