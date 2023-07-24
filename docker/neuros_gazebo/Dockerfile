FROM --platform=linux/amd64 neuros_python
MAINTAINER Lee Perry leeperryis@hotmail.com

RUN apt-get update \
    && apt-get install -y ros-foxy-ros-ign-bridge \
    && apt-get install -y ignition-fortress \
    && rm -rf /var/lib/apt/lists/*
