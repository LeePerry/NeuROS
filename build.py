#!/usr/bin/env python3

import subprocess
import sys

from src.project_config import ProjectConfig
from src.container import Container

def confirmation(question):
    print(question)
    yes = ['yes', 'y', 'yeah']
    no = ['no', 'n', 'nah']
    while True:
        choice = input().lower()
        if choice in yes:
            return True
        elif choice in no:
            return False
        else:
            sys.stdout.write("Please enter 'yes' or 'no'")

def check_docker_installation():
    print("Checking Docker...")
    process = subprocess.run(["docker", "--version"])
    if process.returncode > 0:
        print("Docker not found! " +
              "Please install it and try again.")
        sys.exit(1)
    print("... OK.")

def pull_latest_ros2():
    if confirmation("Allow NeuROS to download the latest ROS2 container? [y/n]"):
        print("Download ROS2 container...")
        process = subprocess.run(["docker", "pull", "osrf/ros:foxy-desktop"])
        if process.returncode > 0:
            print("Failed to download ROS2! " +
                  "Please try again later.")
            sys.exit(1)
    print("... OK.")

def build_neuros(container):
    print("Building workspace...")
    process = container.build_workspace()
    if process.returncode > 0:
        print(f"Failed to build NeuROS: error {process.returncode}")
        sys.exit(1)
    print("... OK.")

def list_neuros_executables(container):
    print("Listing executables...")
    process = container.list_executables()
    if process.returncode > 0:
        print(f"Failed to inspect NeuROS package: error {process.returncode}")
        sys.exit(1)
    print("... OK.")

def main():
    print("-----------------")
    print("Welcome to NeuROS")
    print("-----------------")
    print("")
    check_docker_installation()
    print("")
    pull_latest_ros2()
    print("")
    config = ProjectConfig.build_environment()
    container = Container(config)
    build_neuros(container)
    print("")
    list_neuros_executables(container)
    print("")
    print("Success!")

if __name__ == '__main__':
    main()
