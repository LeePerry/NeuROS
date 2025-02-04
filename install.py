#!/usr/bin/env python3
# Copyright (c) 2023 Lee Perry

"""
This module is responsible for building and installing NeuROS and all of it's
default included software dependencies.
"""

import argparse
import pathlib
import subprocess
import sys

from src.config import ProjectConfig, CommandLineInterface
from src.container import Container

def assume_yes():
    """
    Checks to see if the user specified --yes, indicating that the installation
    process should assume "yes" for all prompts (i.e. automatic full install).
    """
    parser = argparse.ArgumentParser(
        formatter_class=argparse.RawTextHelpFormatter,
        description=CommandLineInterface.banner)
    parser.add_argument('-y', '--yes',
                        help='Automatic installation.',
                        action="store_true")
    return parser.parse_args().yes

def check_docker_installation():
    """
    Checks to see if docker is installed, and if is is not then exit the
    process with error code 1.
    """
    print("Checking Docker...")
    process = subprocess.run(["docker", "--version"])
    if process.returncode > 0:
        print("Docker not found! Please install it and try again.")
        sys.exit(1)
    print("... OK.")

def confirmation(question):
    """
    Display a yes/no prompt to the user, then wait and validate their response.

    Parameters:
        question (str): The prompt to display.

    Returns:
        A boolean, with True indicating that the user responded "yes".
    """
    print(question + " [yes/no]")
    yes = ['yes', 'y', 'yeah']
    no = ['no', 'n', 'nah']
    while True:
        choice = input().lower()
        if   choice in yes: return True
        elif choice in no : return False
        else: sys.stdout.write("Please enter 'yes' or 'no'")

def download_latest_ros2(yes):
    """
    Downloads the latest ROS2 Docker image.

    Parameters:
        yes (boolean): True if we should assume an automatic installation.
    """
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

def _build_image(name, directory="docker"):
    """
    Builds a particular NeuROS Docker image. If the build fails then an
    exception is raised.

    Parameters:
        name (str): The name of the Docker image to build.
        directory (str): The directory that name/Dockerfile can be found in.
    """
    print("")
    print(f"Building and installing Docker image: {name}...")
    print("")
    workspace = pathlib.Path(__file__).parent.resolve()
    process = subprocess.run(["docker", "build", "-t", f"{name}:latest",
                        workspace / directory / name])
    if process.returncode > 0:
        raise Exception(f"Failed to build Docker image {name}: " +
                        f"error {process.returncode}")
    print("... OK.")

def build_docker_images(yes):
    """
    Builds all of the NeuROS Docker images.

    Parameters:
        yes (boolean): True if we should assume an automatic installation.
    """
    if yes or confirmation("Allow NeuROS to build and install custom Docker images?"):
        print("Building and installing NeuROS images...")
        _build_image("neuros_python")
        _build_image("neuros_nest")
        _build_image("neuros_tensorflow")
        _build_image("neuros_gazebo")
    else:
        print("... NeuROS nodes will be limited to existing Docker images.")

def build_neuros():
    """
    Builds the NeuROS ROS2 workspace, which results in a single ROS2 node
    capable of loading any user plugin.

    If there is an error, then exit the process with error code 1.
    """
    print("Building NeuROS...")
    process = Container(ProjectConfig.build_environment()).build_workspace()
    if process.returncode > 0:
        print(f"Failed to build NeuROS: error {process.returncode}")
        sys.exit(1)
    print("... OK.")

def build_examples(yes):
    """
    Builds all of the bundled example projects, including custom Docker
    images.

    If there is an error, then exit the process with error code 1.

    Parameters:
        yes (boolean): True if we should assume an automatic installation.
    """
    if yes or confirmation("Allow NeuROS to build and install examples?"):
        print("Building NeuROS examples...")
        process = Container(ProjectConfig.build_environment()).build_examples()
        if process.returncode > 0:
            print(f"Failed to build examples: error {process.returncode}")
            sys.exit(1)
        _build_image("neuros_whiskeye_snn",
            directory="examples/4_brain_simulation/whiskeye/nodes/snn/docker")
        print("... OK.")
    else:
        print("... Some examples may not function correctly.")

def build_documentation():
    """
    Builds all of the auto-generated html documentation, including PlantUML
    diagrams. (Note that the html can be converted to pdf by running "make pdf"
    in the docs directory)

    If there is an error, then exit the process with error code 1.

    Parameters:
        yes (boolean): True if we should assume an automatic installation.
    """
    print("Building documentation...")
    process = Container(ProjectConfig.build_environment()).build_documentation()
    if process.returncode > 0:
        print(f"Failed to build documentation: error {process.returncode}")
        sys.exit(1)
    print("... OK.")

def main():
    """
    Run the complete build and installation process.

    Finish by prompting the user to run an example project.
    """
    yes = assume_yes()
    print("")
    print(CommandLineInterface.banner)
    print("")
    check_docker_installation()
    print("")
    download_latest_ros2(yes)
    print("")
    build_neuros()
    print("")
    build_docker_images(yes)
    print("")
    build_examples(yes)
    print("")
    build_documentation()
    print("")
    print("================================================================================")
    print("")
    print("")
    print("    Congratulations! You're ready to start using NeuROS.")
    print("")
    print("    Read the documentation:")
    print("        ./docs/build/html/index.html")
    print("")
    print("    Get help with command line options:")
    print("        ./launch.py --help")
    print("")
    print("    Or dive straight in and run a basic example:")
    print("        ./launch.py --project ./examples/1_tick/clock/clock.json")
    print("")
    print("")

if __name__ == '__main__':
    main()
