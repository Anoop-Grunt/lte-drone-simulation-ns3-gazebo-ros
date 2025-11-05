from ubuntu:24.04 as base

env DEBIAN_FRONTEND=noninteractive
user root
workdir /app

run apt-get update
run apt-get install -y sudo
run sudo apt-get -y install curl lsb-release gnupg

copy ./install-gazebo.sh .
run sh install-gazebo.sh

copy ./configure-mesa.sh .
run sh configure-mesa.sh

copy ./install-ros-and-ros-gz-bridge.sh .
run sh install-ros-and-ros-gz-bridge.sh

copy ./install-ninja.sh
run sh install-ninja.sh

run rm -rf /var/lib/apt/lists/*
