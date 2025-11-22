cmake_minimum_required(VERSION 3.8)
project(ros_network)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(ns3_ros_node 
  src/main.cc
  src/ros_node.cc
)

ament_target_dependencies(ns3_ros_node
  rclcpp
  geometry_msgs
  std_msgs
  tf2_msgs
  ros_gz_interfaces
)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  /app/ns3-source/build/include
)



install(TARGETS ns3_ros_node
  DESTINATION lib/${PROJECT_NAME}
)

target_link_libraries(ns3_ros_node
  /app/ns3-source/build/lib/libns3-dev-core-default.so
  /app/ns3-source/build/lib/libns3-dev-network-default.so
  /app/ns3-source/build/lib/libns3-dev-internet-default.so
  /app/ns3-source/build/lib/libns3-dev-mobility-default.so
  /app/ns3-source/build/lib/libns3-dev-spectrum-default.so
  /app/ns3-source/build/lib/libns3-dev-buildings-default.so
  /app/ns3-source/build/lib/libns3-dev-propagation-default.so
  /app/ns3-source/build/lib/libns3-dev-lte-default.so
  /app/ns3-source/build/lib/libns3-dev-csma-default.so
  /app/ns3-source/build/lib/libns3-dev-point-to-point-default.so
  /app/ns3-source/build/lib/libns3-dev-traffic-control-default.so
  /app/ns3-source/build/lib/libns3-dev-energy-default.so
  /app/ns3-source/build/lib/libns3-dev-bridge-default.so
  /app/ns3-source/build/lib/libns3-dev-virtual-net-device-default.so
  /app/ns3-source/build/lib/libns3-dev-applications-default.so
  /app/ns3-source/build/lib/libns3-dev-netanim-default.so
  /app/ns3-source/build/lib/libns3-dev-stats-default.so
  /app/ns3-source/build/lib/libns3-dev-uan-default.so
  /app/ns3-source/build/lib/libns3-dev-fd-net-device-default.so
  /app/ns3-source/build/lib/libns3-dev-antenna-default.so
  /app/ns3-source/build/lib/libns3-dev-lr-wpan-default.so
  /app/ns3-source/build/lib/libns3-dev-wifi-default.so
  /app/ns3-source/build/lib/libns3-dev-config-store-default.so
)

set_target_properties(ns3_ros_node PROPERTIES
    INSTALL_RPATH "/app/ns3-source/build/lib"
    BUILD_WITH_INSTALL_RPATH TRUE
)

ament_package()
