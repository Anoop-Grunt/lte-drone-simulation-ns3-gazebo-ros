xhost +local:docker
docker run -ti \
  -u root:root \
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
  gazebo-with-ros \
  bash -c "source /opt/ros/rolling/setup.bash && ros2 launch ros_gz_sim gz_sim.launch.py gz_args:='empty.sdf' > /tmp/gazebo.log 2>&1 & bash"
