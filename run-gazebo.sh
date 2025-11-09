. ./variables.sh
xhost +local:docker
docker run -ti \
  --rm \
  -u root:root \
  --name gazebo-sim \
  --group-add video \
  --device /dev/dri:/dev/dri \
  -e DISPLAY=$DISPLAY \
  -e XDG_RUNTIME_DIR=/run/user/0 \
  -e QT_X11_NO_MITSHM=1 \
  -e MESA_GL_VERSION_OVERRIDE=4.5 \
  -e MESA_GLSL_VERSION_OVERRIDE=450 \
  -e LIBGL_ALWAYS_INDIRECT=0 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v $(pwd)/sim_and_bridge_clean.launch.py:/tmp/sim_and_bridge.launch.py \
  -v $(pwd)/worlds/world.sdf/:/app/world.sdf \
  -v $(pwd)/debugger_config.config:/app/debugger_config.config \
  -v $(pwd)/network_animations/:/app/network_animations/ \
  gz-ros-harmonic\
  bash -c "mkdir -p /run/user/0 && chmod 0700 /run/user/0 && \
           source /opt/ros/jazzy/setup.bash && \
           source /app/install/setup.bash && \
           ros2 launch /tmp/sim_and_bridge.launch.py  & \
           bash"
