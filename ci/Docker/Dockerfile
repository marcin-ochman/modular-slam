FROM ubuntu:24.04

ENV DEBIAN_FRONTEND noninteractive

# install dependencies via apt
ENV DEBCONF_NOWARNINGS yes
# RUN set -x && \
#   apt-get update -y -qq && \
#   apt-get upgrade -y -qq --no-install-recommends && \
#   : "basic dependencies" && \
#   apt-get install -y -qq \
#     build-essential \
#     pkg-config \
#     cmake \
#     git \
#     wget \
#     curl \
#     tar \
#     unzip && \
#   : "g2o dependencies" && \
#   apt-get install -y -qq \
#     libatlas-base-dev \
#     libsuitesparse-dev \
#     libglew-dev && \
#   : "OpenCV dependencies" && \
#   apt-get install -y -qq \
#     libgtk-3-dev \
#     libjpeg-dev \
#     libpng++-dev \
#     libtiff-dev \
#     libopenexr-dev \
#     libwebp-dev \
#     ffmpeg \
#     libavcodec-dev \
#     libavformat-dev \
#     libavutil-dev \
#     libswscale-dev \
#     pipx \
#     libudev-dev \
#     libva-dev \
#     libvdpau-dev \
#     libx11-xcb-dev libfontenc-dev libxaw7-dev libxkbfile-dev libxmu-dev libxmuu-dev libxpm-dev libxres-dev libxss-dev libxt-dev libxv-dev libxxf86vm-dev libxcb-glx0-dev libxcb-render-util0-dev libxcb-xkb-dev libxcb-icccm4-dev libxcb-image0-dev libxcb-keysyms1-dev libxcb-randr0-dev libxcb-shape0-dev libxcb-sync-dev libxcb-xfixes0-dev libxcb-xinerama0-dev libxcb-dri3-dev libxcb-cursor-dev libxcb-dri2-0-dev libxcb-dri3-dev libxcb-present-dev libxcb-composite0-dev libxcb-ewmh-dev libxcb-res0-dev \
#     libxcb-util-dev libxcb-util0-dev python3-distutils-extra libdouble-conversion-dev\
#     libtbb-dev && \
#   : "backward-cpp dependencies" && \
#   apt install -y -qq binutils-dev && \
#   : "other dependencies" && \
#   apt-get install -y -qq \
#     libyaml-cpp-dev \
#     sqlite3 \
#     libsqlite3-dev && \
#   : "remove cache" && \
#   apt-get autoremove -y -qq && \
#   rm -rf /var/lib/apt/lists/*


RUN set -x && \
  apt-get update -y -qq && \
  apt-get upgrade -y -qq --no-install-recommends && \
  apt-get install -y -qq \
    libudev-dev \
    libva-dev \
    libvdpau-dev \
    libx11-xcb-dev \
    libfontenc-dev \
    libxaw7-dev \
    libxkbfile-dev \
    libxmu-dev \
    libxmuu-dev \
    libxpm-dev \
    libxres-dev \
    libxss-dev \
    libxt-dev \
    libxv-dev \
    libxxf86vm-dev \
    libxcb-glx0-dev \
    libxcb-render-util0-dev \
    libxcb-xkb-dev \
    libxcb-icccm4-dev \
    libxcb-image0-dev \
    libxcb-keysyms1-dev \
    libxcb-randr0-dev \
    libxcb-shape0-dev \
    libxcb-sync-dev \
    libxcb-xfixes0-dev \
    libxcb-xinerama0-dev \
    libxcb-dri3-dev \
    libxcb-cursor-dev \
    libxcb-dri2-0-dev \
    libxcb-dri3-dev \
    libxcb-present-dev \
    libxcb-composite0-dev \
    libxcb-ewmh-dev \
    libxcb-res0-dev \
    libxcb-util-dev \
    libxcb-util0-dev \
    libgl1-mesa-dev \
    xkb-data \
    libxcomposite-dev libxcursor-dev libxdamage-dev libxfixes-dev libxi-dev libxinerama-dev libxrandr-dev libxrender-dev libxtst-dev uuid-dev \
    python3-distutils-extra \
    libdouble-conversion-dev \
    libpcre2-dev \
    build-essential \
    pkg-config \
    cmake \
    pipx && \
  apt-get autoremove -y -qq && \
  rm -rf /var/lib/apt/lists/*

RUN pipx ensurepath
RUN pipx install conan
RUN  echo "eval \"\$(register-python-argcomplete pipx)\"" >> ~/.bashrc

ENV PATH="/root/.local/bin/:${PATH}"
COPY conanfile.py conanfile.py
COPY conan_recipes conan_recipes

RUN apt-get install  -y -qq

RUN /bin/bash -c "conan profile detect"
RUN conan export conan_recipes/dbow3/
RUN /bin/bash -c "conan install -of conan_install --build=missing -o '*:shared=True' ."
RUN /bin/bash -c "conan cache clean -s -b -d -t"

CMD ["/bin/bash"]
