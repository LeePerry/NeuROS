#!/bin/bash
set -e
. "/opt/ros/foxy/setup.bash"
. "/home/neuros/workspace/install/setup.bash"
. "/usr/share/gazebo/setup.sh"
exec "$@"
