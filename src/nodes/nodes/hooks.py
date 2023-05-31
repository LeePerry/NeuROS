# TODO potentially rename package to neuros?
# That way clients will import neuros.hooks, which reads far better

initialise_hooks = []
receive_hooks = []

def neuros_initialise(func):
    initialise_hooks.append(func)
    def process_func(*args, **kwargs):
        return func(*args, **kwargs)
    return process_func

def neuros_receive(name):
    def process_args(func):
        receive_hooks.append((name, func))
        def process_func(*args, **kwargs):
            return func(*args, **kwargs)
        return process_func
    return process_args
