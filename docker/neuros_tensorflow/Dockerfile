FROM --platform=linux/amd64 neuros_python
MAINTAINER Lee Perry leeperryis@hotmail.com

RUN apt-get update \
    && pip3 install tensorflow \
    && rm -rf /var/lib/apt/lists/*
