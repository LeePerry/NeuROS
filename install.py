#!/usr/bin/env python3
# Copyright (c) 2023 Lee Perry

import pathlib
import subprocess
import sys

from src.config import ProjectConfig
from src.container import Container

def check_docker_installation():
    print("Checking Docker...")
    process = subprocess.run(["docker", "--version"])
    if process.returncode > 0:
        print("Docker not found! Please install it and try again.")
        sys.exit(1)
    print("... OK.")

def confirmation(question):
    print(question + " [yes/no]")
    yes = ['yes', 'y', 'yeah']
    no = ['no', 'n', 'nah']
    while True:
        choice = input().lower()
        if   choice in yes: return True
        elif choice in no : return False
        else: sys.stdout.write("Please enter 'yes' or 'no'")

def download_latest_ros2():
    if confirmation("Allow NeuROS to download the latest ROS2 image?"):
        print("Download ROS2 image...")
        process = subprocess.run([
            "docker", "pull", ProjectConfig.default_container])
        if process.returncode > 0:
            print(f"Failed to download ROS2: error {process.returncode}")
            sys.exit(1)
        print("... OK.")
    else:
        print("... NeuROS will attempt to use any existing ROS2 image.")

def build_neuros_docker_images():
    if confirmation("Allow NeuROS to build and install custom Docker images?"):
        print("Building and installing NeuROS images...")
        build = ["docker", "build", "-t"]
        docker_dir = pathlib.Path(__file__).parent.parent.resolve() / "docker"
        subprocess.run(build + ["neuros_python:latest", docker_dir / "neuros_python"])
    else:
        print("... NeuROS nodes will be limited to existing Docker images.")

def build_neuros():
    print("Building NeuROS...")
    process = Container(ProjectConfig.build_environment()).build_workspace()
    if process.returncode > 0:
        print(f"Failed to build NeuROS: error {process.returncode}")
        sys.exit(1)
    print("... OK.")

def list_neuros_executables():
    print("Inspecting NeuROS...")
    process = Container(ProjectConfig.build_environment()).list_executables()
    if process.returncode > 0:
        print(f"Failed to inspect NeuROS package: error {process.returncode}")
        sys.exit(1)
    print("... OK.")

def main():
    print("")
    print("+-------------------------+")
    print("|    Welcome to NeuROS    |")
    print("+-------------------------+")
    print("")
    print("An Integration Framework for Heterogeneous Computational Systems Neuroscience")
    print("")
    print("MIT License")
    print("")
    print("Copyright (c) 2023 Lee Perry")
    print("")
    check_docker_installation()
    print("")
    download_latest_ros2()
    print("")
    build_neuros_docker_images()
    print("")
    build_neuros()
    print("")
    list_neuros_executables()
    print("")
    print("Success!")
    print("")
    print("You're ready to start using NeuROS. Run an example with:")
    print("    ./launch.py --project ./examples/tennis/tennis.json")
    print("")

if __name__ == '__main__':
    main()
