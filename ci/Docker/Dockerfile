FROM archlinux:base-devel


RUN pacman -Sy --noconfirm \
    base-devel \
    boost \
    catch2 \
    cmake \
    doxygen \
    gcc \
    gdb \
    git \
    glu \
    go \
    graphviz \
    htop \
    nlohmann-json \
    opencv \
    python-pip \
    rsync \
    qt5 \
    valgrind \
    vim

RUN useradd builduser -m \
  && passwd -d builduser \
  && cd /home/builduser \
  && (printf 'builduser ALL=(ALL) ALL\n' | tee -a /etc/sudoers)


RUN git clone https://aur.archlinux.org/yay.git yay.git && chown builduser -R yay.git && cd yay.git  && sudo -u builduser bash -c "makepkg -si --noconfirm" && cd - && rm -rf yay.git

USER builduser
RUN yay -S --noconfirm librealsense

USER root
