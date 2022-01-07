FROM ubuntu:21.10

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get -y update && \
    apt-get install -y \
    cmake \
    g++ \
    gcc \
    gdb \
    gdbserver \
    git \
    htop \
    less \
    pip \
    valgrind \
    vim \
    libgtk2.0-dev \
    doxygen \
    graphviz \
    rsync \
    libva-dev \
    libopencv-dev \
    libboost-all-dev \
    nlohmann-json3-dev \
    catch2 \
    libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*