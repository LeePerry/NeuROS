import importlib.util
import os

def dynamic_import(directory, filename):
    module_name = os.path.splitext(filename)[0]
    full_path = os.path.join(directory, filename)
    spec = importlib.util.spec_from_file_location(module_name, full_path)
    impl = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(impl)
