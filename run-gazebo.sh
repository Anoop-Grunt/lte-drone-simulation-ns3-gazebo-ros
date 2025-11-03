xhost +local:docker
docker run -ti \
  --rm\
  -u root:root \
  --name gazebo-sim\
  --group-add video \
  --device /dev/dri:/dev/dri \
  -e DISPLAY=$DISPLAY \
  -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
  -e QT_X11_NO_MITSHM=1 \
  -e MESA_GL_VERSION_OVERRIDE=4.5 \
  -e MESA_GLSL_VERSION_OVERRIDE=450 \
  -e LIBGL_ALWAYS_INDIRECT=0 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v $XDG_RUNTIME_DIR:$XDG_RUNTIME_DIR \
  -v $(pwd)/sim_and_bridge_clean.launch.py:/tmp/sim_and_bridge.launch.py:ro \
  gazebo-with-ros \
  bash -c "source /opt/ros/rolling/setup.bash && ros2 launch /tmp/sim_and_bridge.launch.py > /tmp/gazebo.log 2>&1 & bash"
