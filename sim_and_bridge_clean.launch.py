import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # --- 1. Paths and Arguments ---
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    world_sdf_file = LaunchConfiguration(
        'world_sdf_file',
        default='/app/world.sdf'
    )
    
    # --- 2. Launch Gazebo Sim with specified world ---
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': [world_sdf_file]
        }.items()
    )
    
    # --- 3. Bridge for /X3/cmd_vel ---
    cmd_vel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='cmd_vel_bridge',
        output='screen',
        arguments=[
            '/X3/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist'
        ]
    )
    
    # --- 4. Bridge for Gazebo pose updates ---
    pose_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='pose_bridge',
        output='screen',
        arguments=[
            '/model/X3/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
        ]
    )
    
    # --- 5. Launch NS-3 ROS node ---
    ns3_node = Node(
        package='ros_network',
        executable='ns3_ros_node',
        name='ns3_ros_node',
        output='screen',
        emulate_tty=True
    )
    
    # --- 6. Launch Remote Server Node (Quadcopter Pilot) ---
    quadcopter_pilot = Node(
        package='remote_server_nodes',
        executable='quadcopter_pilot',
        name='quadcopter_pilot',
        output='screen',
        emulate_tty=True
    )
    
    # --- 7. Assemble Launch Description ---
    return LaunchDescription([
        DeclareLaunchArgument(
            'world_sdf_file',
            default_value='/app/world.sdf',
            description='SDF file to load in Gazebo Sim'
        ),
        gazebo_sim,
        cmd_vel_bridge,
        pose_bridge,
        ns3_node,
        quadcopter_pilot
    ])
