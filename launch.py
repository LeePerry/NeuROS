#!/usr/bin/env python3

from multiprocessing import Process

from src.cli import parse_cli_args
from src.config import Config
from src.container import Container

def containerised_node(config, name):
    Container(config).run_node(name)

def main():
    args = parse_cli_args()
    config = Config.from_file(args.project_path)
    Container(config).build_workspace()
    nodes = []
    for name in args.node:
        node = Process(target=containerised_node, args=(config, name))
        nodes.append(node)
        node.start()
    for node in nodes:
        node.join()

if __name__ == '__main__':
    main()
