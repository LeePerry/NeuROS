FROM neuros_python
MAINTAINER Lee Perry leeperryis@hotmail.com

RUN apt update \
    && apt -y install software-properties-common \
    && add-apt-repository -y ppa:nest-simulator/nest \
    && apt install -y nest \
    && rm -rf /var/lib/apt/lists/*
