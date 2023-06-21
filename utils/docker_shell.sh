#!/bin/bash
set -e
docker run -it \
    --volume "$HOME/Code/NeuROS:/home/neuros/workspace" \
    --user=$(id -u $USER):$(id -g $USER) \
    --env="DISPLAY" \
    --device="/dev/dri:/dev/dri" \
    --volume="$HOME:$HOME:rw" \
    --volume="/etc/group:/etc/group:ro" \
    --volume="/etc/passwd:/etc/passwd:ro" \
    --volume="/etc/shadow:/etc/shadow:ro" \
    --volume="/etc/sudoers.d:/etc/sudoers.d:ro" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --init --rm \
    $1 bash
