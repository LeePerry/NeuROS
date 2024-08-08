FROM --platform=linux/amd64 neuros_python
MAINTAINER Lee Perry leeperryis@hotmail.com

RUN apt-get update \
    && apt-get install wget libgsl-dev -y \
    && wget https://github.com/nest/nest-simulator/archive/refs/tags/v2.18.0.tar.gz \
    && tar -xf v2.18.0.tar.gz \
    && mkdir nest-simulator-2.18.0/build \
    && cd nest-simulator-2.18.0/build \
    && pip3 install cython \
    && cmake -DCMAKE_INSTALL_PREFIX:PATH=/opt/ros/humble -Dwith-python=3 .. \
    && make \
    && make install \
    && sed -i 's/ast.Module(body=body)/ast.Module(body=body, type_ignores=[])/g' /opt/ros/humble/lib/python3.10/site-packages/nest/import_libs.py \
    && rm -rf /var/lib/apt/lists/*
