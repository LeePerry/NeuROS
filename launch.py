#!/usr/bin/env python3

from multiprocessing import Process

from src.cli import parse_cli_args
from src.project_config import ProjectConfig
from src.container import Container

def containerised_node(config, name):
    Container(config).run_node(name)

def main():
    args = parse_cli_args()
    config = ProjectConfig.from_file(args.project_path)
    nodes = [Process(target=containerised_node,
                     args=(config, name))
                     for name in args.node]
    for node in nodes: node.start()
    for node in nodes: node.join()

if __name__ == '__main__':
    main()
