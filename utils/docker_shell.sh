#!/usr/bin/env bash
set -e

# osx
#docker run -it \
#    --volume /tmp/.X11-unix:/tmp/.X11-unix \
#    --volume "/Users/leeperry/Documents/Dissertation/NeuROS:/home/neuros/workspace" \
#    --env DISPLAY=host.docker.internal:0 \
#    --platform=linux/amd64 \
#    --init --rm \
#    $1 bash

# linux
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

# docker run --gpus all -it --privileged -e DISPLAY="$DISPLAY" --network host --name ubuntu20_ign --volume="$HOME/.Xauthority:/root/.Xauthority:rw" -v /tmp/.X11-unix:/tmp/.X11-unix -v /home/tomoyafujita/DVT/docker_ws:/root/docker_ws nvidia/opengl:base