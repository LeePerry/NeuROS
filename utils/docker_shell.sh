#!/usr/bin/env bash

docker run -it \
    --volume "$HOME/Code/NeuROS:/home/neuros/workspace" \
    --volume "$HOME/Code/NeuROS/examples/4_brain_simulation/whiskeye:/home/neuros/project" \
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
    --platform=linux/amd64 \
    $1 bash
