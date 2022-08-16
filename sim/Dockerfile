FROM osrf/ros:foxy-desktop
LABEL maintainer="James Goppert <james.goppert@gmail.com>"

# install other dependencies
RUN apt-get update && apt-get -y --quiet --no-install-recommends install \
  cmake \
  build-essential \
  ignition-fortress \
  git \
  mesa-utils \
  libnvidia-gl-515-server \
  ros-foxy-mavlink \
  python3-rospkg \
  xterm \
  terminator \
  bash-completion \
  libcanberra-gtk-module \
  && apt-get -y autoremove \
	&& apt-get clean autoclean \
	&& rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*

# px4 simulation gazebo
RUN . /opt/ros/foxy/setup.sh \
  && git clone https://github.com/jgoppert/px4-simulation-ignition.git --depth 1 -b gazebo6 /tmp/px4-simulation-ignition \
  && cd /tmp/px4-simulation-ignition \
  && mkdir build && cd build \
  && cmake .. && make && cpack -G DEB \
  && dpkg -i *.deb

# setup user
RUN groupadd -r -g 110 render \
 && adduser --disabled-password --gecos '' docker \
 && adduser docker sudo \
 && adduser docker render \
 && echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

# create setting directory for ignition gazebo
RUN mkdir -p /home/docker/.ignition \
  && chown -R docker:docker /home/docker/.ignition

USER docker
WORKDIR /home/docker

RUN mkdir -p /tmp/runtime-docker && chmod 700 /tmp/runtime-docker
ENV XDG_RUNTIME_DIR=/tmp/runtime-docker

# vim: set et fenc=utf-8 ff=unix ft=dockerfile sts=0 sw=2 ts=2 :