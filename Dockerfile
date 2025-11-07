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

copy ./install-ninja.sh .
run sh install-ninja.sh

copy ./install-git.sh .
run sh ./install-git.sh

copy ./variables.sh .
copy ./build-netsim.sh .
copy ./install-ns3.sh .
run sh install-ns3.sh .

copy ./download-world.sh .
run sh ./download-world.sh

copy  ./install-colcon.sh .
run sh ./install-colcon.sh

copy ./setup-ros-network-node.sh .
run bash ./setup-ros-network-node.sh 

copy ./ros_network/CMakeLists.txt ./ros_network/
copy ./scripts/main.cc ./ros_network/src/
copy ./build-ros-nodes.sh .
run bash ./build-ros-nodes.sh 

run rm -rf /var/lib/apt/lists/*


