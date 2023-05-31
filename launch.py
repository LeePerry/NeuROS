#!/usr/bin/env python3

import multiprocessing

from src.cli import parse_cli_args
from src.project_config import ProjectConfig
from src.container import Container

# launch -> multiprocess -> subprocess -> node

def launch_node(config, name):
    Container(config).run_node(name)

def main():
    args = parse_cli_args()
    config = ProjectConfig.from_file(args.project_path)
    print(f"Launching {config.name}...")
    nodes = [multiprocessing.Process(target=launch_node,
                                     args=(config, name))
                                     for name in args.node]
    for node in nodes: node.start()
    for node in nodes: node.join()

if __name__ == '__main__':
    main()
