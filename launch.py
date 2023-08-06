#!/usr/bin/env python3
# Copyright (c) 2023 Lee Perry

import multiprocessing

from src.config import ProjectConfig
from src.container import Container

def launch_node(config, name):
    Container(config).run_node(name)

def main():
    args = ProjectConfig.cli_args()
    config = ProjectConfig.from_file(args.project_path)
    nodes = [multiprocessing.Process(target=launch_node, args=(config, name))
            for name in (args.node if args.node else config.nodes.keys())]
    for node in nodes: node.start()
    for node in nodes: node.join()

if __name__ == '__main__':
    main()
