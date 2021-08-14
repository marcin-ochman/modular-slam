FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get -y update && \
    apt-get install -y \
    cmake \
    g++ \
    gcc \
    gdb \
    gdbserver \
    htop \
    less \
    pip \
    valgrind \
    vim \
    libgtk2.0-dev \
    doxygen \
    graphviz \
    && rm -rf /var/lib/apt/lists/*

RUN pip install \
    conan

COPY conanfile.txt conanfile.txt
RUN conan install conanfile.txt
