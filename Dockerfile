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

copy ./ros_network/setup-ros-network-node.sh .
run bash ./setup-ros-network-node.sh 

copy ./ros_network/CMakeLists.txt ./ros_network/
copy ./ros_network/main.cc ./ros_network/src/
copy ./ros_network/ros_node.cc ./ros_network/src/
copy ./ros_network/ros_node.h ./ros_network/include/

copy ./remote_server_nodes/setup_remote_server_nodes.sh .
run bash ./setup_remote_server_nodes.sh

copy ./remote_server_nodes/quadcopter_pilot.py ./remote_server_nodes/remote_server_nodes/
copy ./remote_server_nodes/follow_cam.py ./remote_server_nodes/remote_server_nodes/
copy ./remote_server_nodes/setup.py ./remote_server_nodes/

COPY ./remote_server_nodes/autonomous_pilot.py ./remote_server_nodes/remote_server_nodes/
COPY ./remote_server_nodes/diagonal_pilot.py ./remote_server_nodes/remote_server_nodes/
COPY ./remote_server_nodes/rl_pilot.py ./remote_server_nodes/remote_server_nodes/
COPY ./remote_server_nodes/rl_test.py ./remote_server_nodes/remote_server_nodes/
COPY ./remote_server_nodes/rl_plotter.py ./remote_server_nodes/remote_server_nodes/

copy ./build-ros-nodes.sh .
run bash ./build-ros-nodes.sh 

copy ./signal_strength/install_custom_gz_plugin.sh .
copy ./signal_strength/ ./signal_strength/
run  sh ./install_custom_gz_plugin.sh

run rm -rf /var/lib/apt/lists/*

