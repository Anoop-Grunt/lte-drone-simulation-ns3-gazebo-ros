
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

apt-get update && apt-get install -y \
    ros-jazzy-desktop \
    ros-jazzy-ros-gz \

echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash.bashrc
