find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

file(GLOB ros_sources CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/*.cc)

build_exec(
  EXECNAME robot_sim
  EXECNAME_PREFIX scratch_ros_scripts_
  SOURCE_FILES "${ros_sources}"
  LIBRARIES_TO_LINK "${ns3-libs}" "${ns3-contrib-libs}"
  EXECUTABLE_DIRECTORY_PATH ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/scratch/ros_scripts/
)

target_include_directories(scratch_ros_scripts_robot_sim PRIVATE 
  ${rclcpp_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
)

target_link_libraries(scratch_ros_scripts_robot_sim
  ${libcore}
  ${libnetwork}
  ${libinternet}
  ${liblte}
  ${libapplications}
  ${libpoint-to-point}
  ${libmobility}
  ${rclcpp_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
)

ament_target_dependencies(scratch_ros_scripts_robot_sim rclcpp geometry_msgs) 

