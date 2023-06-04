#!/usr/bin/env python3
# Copyright (c) 2023 Lee Perry

import multiprocessing

from src.cli import CommandLineInterface
from src.project_config import ProjectConfig
from src.container import Container

def launch_node(config, name):
    Container(config).run_node(name)

def main():
    args = CommandLineInterface.from_command_line()
    config = ProjectConfig.from_file(args.project_path)
    print(f"Launching {config.get_name()}...")
    nodes = [multiprocessing.Process(target=launch_node, args=(config, name))
            for name in (args.node if args.node else config.get_all_nodes())]
    for node in nodes: node.start()
    for node in nodes: node.join()

if __name__ == '__main__':
    main()
