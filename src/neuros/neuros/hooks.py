# Copyright (c) 2023 Lee Perry

class Hooks:

    on_initialise = []
    on_tick = []
    on_receive_any = []
    on_receive_all = []

    @classmethod
    def no_args(cls, hooks, func):
        hooks.append(func)
        def process_func(node):
            return func(node)
        return process_func

    @classmethod
    def single_arg(cls, hooks, arg):
        def process_args(func):
            hooks.append((arg, func))
            def process_func(node):
                return func(node)
            return process_func
        return process_args

    @classmethod
    def filter_by_connection(cls, hooks, connection):
        return [h for n, h in hooks if connection in n]

def neuros_initialise(func):
    return Hooks.no_args(Hooks.on_initialise, func)

def neuros_tick(seconds):
    return Hooks.single_arg(Hooks.on_tick, seconds)

def neuros_receive_any(*connection_names):
    return Hooks.single_arg(Hooks.on_receive_any, connection_names)

def neuros_receive_all(*connection_names):
    return Hooks.single_arg(Hooks.on_receive_all, connection_names)

# TODO consider
#
# neuros_receive_accumulate(connection, count)
