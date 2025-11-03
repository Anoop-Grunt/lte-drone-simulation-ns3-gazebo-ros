import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # --- 1. Define Arguments and Paths ---
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    world_sdf_file = LaunchConfiguration('world_sdf_file', default='empty.sdf')
    
    # The topic name used in Gazebo Sim
    gz_clock_topic = '/world/empty/clock'

    # --- 2. Launch Gazebo Sim ---

    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': [' -r ', world_sdf_file]}.items() 
    )

    # --- 3. Launch Parameter Bridge for Clock (The Fix is Here) ---
    
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        # Corrected Argument Format: GZ_TOPIC@ROS_MSG_TYPE[GZ_MSG_TYPE
        # The [ bracket forces a GZ->ROS bridge and should map to the standard ROS topic name if none is explicitly provided.
        arguments=[
            f'{gz_clock_topic}@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ]
    )
    
    # --- 4. Assemble the Launch Description ---

    return LaunchDescription([
        
        gazebo_sim,
        
        gz_bridge,
        
        DeclareLaunchArgument(
            'world_sdf_file',
            default_value='empty.sdf',
            description='SDF file to load in Gazebo Sim'
        ),
    ])
