import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # --- 0. Set environment variables ---
    gz_gui_plugin_path = SetEnvironmentVariable(
        name='GZ_GUI_PLUGIN_PATH',
        value='/usr/local/lib/gz-gui-8/plugins:/usr/lib/x86_64-linux-gnu/gz-gui-8/plugins'
    )

    # --- 1. Paths and Arguments ---
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    world_sdf_file = LaunchConfiguration(
        'world_sdf_file',
        default='/app/world.sdf'
    )
    
    # --- 2. Launch Gazebo Sim with proper command line flags ---
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': [world_sdf_file, ' -r --gui-config ./debugger_config.config']
        }.items()
    )
    
    # --- Rest of your nodes... ---
    cmd_vel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='cmd_vel_bridge',
        output='screen',
        arguments=[
            '/X3/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist'
        ]
    )
    
    pose_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='pose_bridge',
        output='screen',
        arguments=[
            '/model/X3/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
        ]
    )

    cell_id_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='cell_id_bridge',
        output='screen',
        arguments=[
            '/current_cell_id@std_msgs/msg/UInt32]gz.msgs.UInt32'
        ]
    )
    
    rsrp_bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    name='rsrp_bridge',
    output='screen',
    arguments=[
        '/rsrp_values@ros_gz_interfaces/msg/Float32Array]gz.msgs.Float_V'

    ]
)
    ns3_node = Node(
        package='ros_network',
        executable='ns3_ros_node',
        name='ns3_ros_node',
        output='screen',
        emulate_tty=True
    )
    
    quadcopter_pilot = Node(
        package='remote_server_nodes',
        executable='quadcopter_pilot',
        name='quadcopter_pilot',
        output='screen',
        emulate_tty=True
    )
    
    camera_follower = Node(
        package='remote_server_nodes',
        executable='camera_follower',
        name='camera_follower',
        output='screen',
        emulate_tty=True
    )
    
    # --- 7. Assemble Launch Description ---
    return LaunchDescription([
        gz_gui_plugin_path,
        DeclareLaunchArgument(
            'world_sdf_file',
            default_value='/app/world.sdf',
            description='SDF file to load in Gazebo Sim'
        ),
        gazebo_sim,
        cmd_vel_bridge,
        pose_bridge,
        rsrp_bridge,
        cell_id_bridge,
        ns3_node,
        quadcopter_pilot,
        #camera_follower
    ])
