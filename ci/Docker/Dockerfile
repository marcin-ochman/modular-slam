FROM ubuntu:18.04

RUN apt-get -y update && \
    apt-get install -y \
    cmake \
    g++-8 \
    gcc-8 \
    gdb \
    gdbserver \
    htop \
    less \
    python3 \
    python3-pip \
    valgrind \
    vim \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install \
    conan