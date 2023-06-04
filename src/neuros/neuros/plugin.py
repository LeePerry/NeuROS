# Copyright (c) 2023 Lee Perry

import importlib.util
import os

import std_msgs.msg

def plugin_import(directory, filename):
    module_name = os.path.splitext(filename)[0]
    full_path = os.path.join(directory, filename)
    spec = importlib.util.spec_from_file_location(module_name, full_path)
    impl = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(impl)

# TODO need a version of this that performs an import first
def plugin_packet_type(name):
    try:
        obj = eval(name)
        if isinstance(obj, type):
            return obj
    except Exception:
        pass
    raise Exception(f"Invalid message type: {name}!") from None
