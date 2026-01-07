#!/usr/bin/bash

APT_DEPENDENCIES="cmake \
                  g++ \
                  libblas-dev \
                  libboost-serialization-dev \
                  libc++-dev \
                  libegl1-mesa-dev \
                  libeigen3-dev \
                  libepoxy-dev \
                  libg2o-dev \
                  libgl1-mesa-dev \
                  libgles2-mesa-dev \
                  libglew-dev \
                  liblapack-dev \
                  libopencv-dev \
                  nasm \
                  libwayland-dev \
                  libx11-dev \
                  libxkbcommon-dev \
                  ninja-build \
                  wayland-protocols"


myarg=$1
if [[ "$myarg" = "--deps" ]]; then
    echo $APT_DEPENDENCIES
    exit 0
fi


sudo apt-get update && \
sudo apt-get install --no-install-recommends -y $APT_DEPENDENCIES
