FROM --platform=linux/amd64 neuros_python
MAINTAINER Lee Perry leeperryis@hotmail.com

RUN apt-get update \
    && apt-get -y install software-properties-common \
    && add-apt-repository -y ppa:nest-simulator/nest \
    && apt-get install -y nest \
    && rm -rf /var/lib/apt/lists/*
