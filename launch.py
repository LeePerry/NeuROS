#!/usr/bin/env python3
# Copyright (c) 2023 Lee Perry

"""
This module is responsible for launching a NeuROS project.
"""

import os
import psutil
import signal
import threading
import time

from src.config import ProjectConfig
from src.container import Container

def stop_all_children():
    """
    This function sends a SIGTERM signal to all of the NeuROS child processes.

    This is required as only the foreground process group receives the Ctrl+C
    signal when a user attempts to stop the session. The others must be closed
    manually, as follows.
    """
    for child in psutil.Process().children(recursive=True):
        os.kill(child.pid, signal.SIGTERM)

def start_recording(config, topics):
    """
    This function launches the "record topics" container and blocks until
    it returns.

    Parameters:
        config (ProjectConfig) : The NeuROS project config.
        topics (list) : A list of topics, represented as strings, which should
                        be recorded.
    """
    Container(config).record_topics(topics)
    stop_all_children()

def start_monitoring(config):
    """
    This function will the "monitor system load" process and block until it
    completes.

    Parameters:
        config (ProjectConfig) : The NeuROS project config.
    """
    Container(config).docker_command("src/monitor.py",
                                     container="neuros_python")
    stop_all_children()

def launch_third_party_utility(config, utility):
    """
    This function will launch a 3rd party utility process and block until it
    completes.

    Parameters:
        config (ProjectConfig) : The NeuROS project config.
    """
    time.sleep(3)
    Container(config).docker_command(utility,
                                     container="neuros_gazebo")
    stop_all_children()

def launch_node(config, name, verbose, domain_id):
    """
    This function will launch a NeuROS node and block until it completes.

    Parameters:
        config (ProjectConfig) : The NeuROS project config.
        name (str) : The uniquely identifying name of the node to launch.
        domain_id (int) : The domain ID that this node should belong to.
                          Use this to logically separate running NeuROS
                          projects. (Defaults to 0.)
    """
    Container(config).run_node(name, verbose, domain_id)
    stop_all_children()

def main():
    """
    The main NeuROS entry point that is invoked during launch.
    """
    args = ProjectConfig.cli_args()
    config = ProjectConfig.from_file(args.project)

    utilities = []
    if args.record:
        utilities.append(threading.Thread(
            target=start_recording,
            args=(config, args.topic)))
    if args.monitor_system_load:
        utilities.append(threading.Thread(
            target=start_monitoring,
            args=(config,)))
    if args.node_graph:
        utilities.append(threading.Thread(
            target=launch_third_party_utility,
            args=(config,"rqt_graph")))
    if args.visualisations:
        utilities.append(threading.Thread(
            target=launch_third_party_utility,
            args=(config,"rviz2")))
    nodes = [threading.Thread(
            target=launch_node,
            args=(config, name, args.verbose, args.domain_id))
        for name in (args.node if args.node else config.nodes.keys())]

    for utility in utilities: utility.start()
    for node in nodes: node.start()
    for node in nodes: node.join()
    for utility in utilities: utility.join()

if __name__ == '__main__':
    try: main()
    finally: os.system("stty sane")
