#!/usr/bin/env python3

import multiprocessing

import src.cli
import src.config
import src.container

def main(config, name):
    src.container.Container(config, name).run_and_wait()

if __name__ == '__main__':
    args = src.cli.parse_cli_args()
    config = src.config.Config.from_file(args.project_path)
    nodes = []
    for name in args.node:
        node = multiprocessing.Process(target=main, args=(config, name))
        nodes.append(node)
        node.start()
    for node in nodes:
        node.join()
