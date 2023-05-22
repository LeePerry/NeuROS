FROM osrf/ros:humble-desktop
MAINTAINER Lee Perry leeperryis@hotmail.com
WORKDIR "/home/neuros/workspace"

RUN apt-get update \
    && apt-get install -y apt-utils \
    && echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections \
    && apt-get install -y dialog \
    && apt-get -y install pip \
    && pip3 install sphinx \
    && rm -rf /var/lib/apt/lists/*

COPY ./setup_neuros.sh /home/neuros
ENTRYPOINT ["/home/neuros/setup_neuros.sh"]
