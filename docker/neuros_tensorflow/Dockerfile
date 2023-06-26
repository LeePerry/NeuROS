FROM --platform=linux/amd64 neuros_python
MAINTAINER Lee Perry leeperryis@hotmail.com

RUN apt update \
    && apt -y install pip \
    && pip install tensorflow \
    && rm -rf /var/lib/apt/lists/*
