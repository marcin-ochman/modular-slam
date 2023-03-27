FROM archlinux:base-devel

RUN pacman -Sy --noconfirm archlinux-keyring && pacman -Su --noconfirm

RUN pacman -Syu --noconfirm \
    base-devel \
    boost \
    catch2 \
    cmake \
    doxygen \
    gcc \
    gdb \
    git \
    glu \
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
    qt6 \
    valgrind \
    vtk \
    vim

RUN useradd builduser -m \
  && passwd -d builduser \
  && cd /home/builduser \
  && (printf 'builduser ALL=(ALL) ALL\n' | tee -a /etc/sudoers)

COPY dbow3.patch /modular-slam-data/dbow3.patch

RUN git clone https://aur.archlinux.org/yay.git yay.git && chown builduser -R yay.git && cd yay.git  && sudo -u builduser bash -c "makepkg -si --noconfirm" && cd - && rm -rf yay.git
RUN git clone https://github.com/rmsalinas/DBow3.git && cd DBow3 && mkdir build && git apply /modular-slam-data/dbow3.patch && cd build && cmake .. -DCMAKE_INSTALL_PREFIX=/opt/dbow3 -DBUILD_UTILS=OFF && make install && cd -

USER builduser
RUN yay -Syu --noconfirm librealsense ceres-solver spdlog trompeloeil-git

USER root
