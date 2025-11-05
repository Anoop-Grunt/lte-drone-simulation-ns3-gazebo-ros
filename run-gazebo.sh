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
  -v $(pwd)/sim_and_bridge_clean.launch.py:/tmp/sim_and_bridge.launch.py:ro \
  -v $(pwd)/scripts/:/app/${NS3_SOURCE_DIR}/scratch/ros_scripts/ \
  gz-ros-harmonic\
  bash -c "mkdir -p /run/user/0 && chmod 0700 /run/user/0 && \
           source /opt/ros/jazzy/setup.bash && \
           ${NS3_SOURCE_DIR}/ns3 configure -G Ninja \
             --enable-examples \
             --enable-modules='core;network;internet;mobility;lte;propagation;spectrum;buildings;config-store;netanim;'\
             --disable-modules='' \
             --disable-tests --disable-gsl --disable-mpi  && \
           ${NS3_SOURCE_DIR}/ns3 build && \
           ros2 launch /tmp/sim_and_bridge.launch.py > /tmp/gazebo.log 2>&1 & \
           bash"
