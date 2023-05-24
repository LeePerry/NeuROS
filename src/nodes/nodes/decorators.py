
class DecoratoredFunctions:

    def __init__(self):
        self._initialise_hooks = []
        self._connection_hooks = {}

    def register_initialise_hook(self, func):
        print(f"Registered for initialisation: {func.__name__}")
        self._initialise_hooks.append(func)

    def register_connection_hook(self, name, func):
        print(f"Registered connection {name}: {func.__name__}")
        if name not in self._connection_hooks:
            self._connection_hooks[name] = []
        self._connection_hooks[name].append(func)

decorated_functions = DecoratoredFunctions()

def neuros_initialise(func):
    decorated_functions.register_initialise_hook(func)
    def process_func(*args, **kwargs):
        return func(*args, **kwargs)
    return process_func

def neuros_connection(name):
    def process_args(func):
        decorated_functions.register_connection_hook(name, func)
        def process_func(*args, **kwargs):
            return func(*args, **kwargs)
        return process_func
    return process_args
