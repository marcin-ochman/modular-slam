FROM archlinux:base-devel

RUN pacman -Sy --noconfirm archlinux-keyring && pacman -Su --noconfirm
RUN pacman -Syu --noconfirm \
    base-devel \
    boost \
    cmake \
    doxygen \
    eigen \
    fmt \
    gcc \
    git \
    glu \
    glew \
    glm \
    go \
    graphviz \
    hdf5 \
    htop \
    nlohmann-json \
    opencv \
    python-pip \
    rsync \
    qt5 \
    vtk

RUN useradd builduser -m \
  && passwd -d builduser \
  && cd /home/builduser \
  && (printf 'builduser ALL=(ALL) ALL\n' | tee -a /etc/sudoers)

RUN git clone https://aur.archlinux.org/yay.git yay.git && chown builduser -R yay.git && cd yay.git  && sudo -u builduser bash -c "makepkg -si --noconfirm" && cd - && rm -rf yay.git

ARG PANGOLIN_COMMIT=v0.6
WORKDIR /tmp
RUN set -x && \
  git clone https://github.com/stevenlovegrove/Pangolin.git && \
  cd Pangolin && \
  git checkout ${PANGOLIN_COMMIT} && \
  mkdir -p build && \
  cd build && \
  cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/opt/pangolin \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_PANGOLIN_DEPTHSENSE=OFF \
    -DBUILD_PANGOLIN_FFMPEG=OFF \
    -DBUILD_PANGOLIN_LIBDC1394=OFF \
    -DBUILD_PANGOLIN_LIBJPEG=OFF \
    -DBUILD_PANGOLIN_LIBOPENEXR=OFF \
    -DBUILD_PANGOLIN_LIBPNG=OFF \
    -DBUILD_PANGOLIN_LIBTIFF=OFF \
    -DBUILD_PANGOLIN_LIBUVC=OFF \
    -DBUILD_PANGOLIN_LZ4=OFF \
    -DBUILD_PANGOLIN_OPENNI=OFF \
    -DBUILD_PANGOLIN_OPENNI2=OFF \
    -DBUILD_PANGOLIN_PLEORA=OFF \
    -DBUILD_PANGOLIN_PYTHON=OFF \
    -DBUILD_PANGOLIN_TELICAM=OFF \
    -DBUILD_PANGOLIN_UVC_MEDIAFOUNDATION=OFF \
    -DBUILD_PANGOLIN_V4L=OFF \
    -DBUILD_PANGOLIN_ZSTD=OFF \
    .. && \
  make -j && \
  make install && \
  cd /tmp && \
  rm -rf *
ENV Pangolin_DIR=${CMAKE_INSTALL_PREFIX}/lib/cmake/Pangolin

RUN git clone --recursive https://github.com/UZ-SLAMLab/ORB_SLAM3.git && cd ORB_SLAM3 && ./build.sh


USER root
