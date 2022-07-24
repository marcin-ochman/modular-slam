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
    htop \
    nlohmann-json \
    opencv \
    python-pip \
    rsync \
    qt5 \
    qt6 \
    valgrind \
    vim

RUN useradd builduser -m \
  && passwd -d builduser \
  && cd /home/builduser \
  && (printf 'builduser ALL=(ALL) ALL\n' | tee -a /etc/sudoers)


RUN git clone https://aur.archlinux.org/yay.git yay.git && chown builduser -R yay.git && cd yay.git  && sudo -u builduser bash -c "makepkg -si --noconfirm" && cd - && rm -rf yay.git

USER builduser
RUN yay -Syu --noconfirm librealsense ceres-solver trompeloeil-git

USER root
