#!/usr/bin/env python3
# Copyright (c) 2023 Lee Perry

import argparse
import pathlib
import subprocess
import sys

from src.config import ProjectConfig
from src.container import Container

def assume_yes():
    parser = argparse.ArgumentParser(
        formatter_class=argparse.RawTextHelpFormatter,
        description="NeuROS\n\n" +
                "An Integration Framework for Heterogenous Systems Neuroscience")
    parser.add_argument('-y', '--yes',
                        help='Automatic installation.',
                        action="store_true")
    return parser.parse_args().yes

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

def download_latest_ros2(yes):
    if yes or confirmation("Allow NeuROS to download the latest ROS2 Docker image?"):
        print("Downloading ROS2 Docker image...")
        process = subprocess.run([
            "docker", "pull", ProjectConfig.default_container])
        if process.returncode > 0:
            print(f"Failed to download ROS2: error {process.returncode}")
            sys.exit(1)
        print("... OK.")
    else:
        print("... NeuROS will attempt to use any existing ROS2 image.")

def build_docker_images(yes):
    if yes or confirmation("Allow NeuROS to build and install custom Docker images?"):
        print("Building and installing NeuROS images...")
        def _image(name):
            print("")
            print(f"Building and installing Docker image: {name}...")
            print("")
            workspace = pathlib.Path(__file__).parent.resolve()
            process = subprocess.run(["docker", "build", "-t", f"{name}:latest",
                                workspace / "docker" / name])
            if process.returncode > 0:
                raise Exception(f"Failed to build Docker image {name}: " +
                                f"error {process.returncode}")
            print("... OK.")
        _image("neuros_python")
        #_image("neuros_nest")
        #_image("neuros_tensorflow")
        _image("neuros_gazebo")
    else:
        print("... NeuROS nodes will be limited to existing Docker images.")

def build_neuros():
    print("Building NeuROS...")
    process = Container(ProjectConfig.build_environment()).build_workspace()
    if process.returncode > 0:
        print(f"Failed to build NeuROS: error {process.returncode}")
        sys.exit(1)
    print("... OK.")

def main():
    yes = assume_yes()
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
    download_latest_ros2(yes)
    print("")
    build_neuros()
    print("")
    build_docker_images(yes)
    print("")
    print("Success!")
    print("")
    print("You're ready to start using NeuROS. Run an example with:")
    print("    ./launch.py --project_path ./examples/tennis/tennis.json")
    print("")

if __name__ == '__main__':
    main()
