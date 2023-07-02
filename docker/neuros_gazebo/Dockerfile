FROM --platform=linux/amd64 neuros_python
MAINTAINER Lee Perry leeperryis@hotmail.com

RUN apt-get update \
    && apt-get install -y ros-foxy-gazebo-ros-pkgs \
    && apt-get install -y python3-ignition-gazebo6 \
    #&& apt install -y libgl1-mesa-glx libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*

COPY ./setup_gazebo.sh /home/neuros
ENTRYPOINT ["/home/neuros/setup_gazebo.sh"]
