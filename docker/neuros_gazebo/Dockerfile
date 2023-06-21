FROM neuros_python
MAINTAINER Lee Perry leeperryis@hotmail.com

RUN apt update \
    && apt install -y apt-utils \
    && apt install -y ros-foxy-gazebo-ros-pkgs \
    #&& apt install -y libgl1-mesa-glx libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*

COPY ./setup_gazebo.sh /home/neuros
ENTRYPOINT ["/home/neuros/setup_gazebo.sh"]
