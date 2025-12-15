import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import PythonExpression
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # --- 0. Set environment variables ---
    gz_gui_plugin_path = SetEnvironmentVariable(
        name='GZ_GUI_PLUGIN_PATH',
        value='/usr/local/lib/gz-gui-8/plugins:/usr/lib/x86_64-linux-gnu/gz-gui-8/plugins'
    )

    # --- 1. Paths and Arguments ---
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # Control mode argument: 'manual', 'autonomous', 'diagonal', 'rl', or 'rl_test'
    control_mode_arg = DeclareLaunchArgument(
        'control_mode',
        default_value='manual',
        description='Control mode: manual (keyboard), autonomous (waypoint), diagonal (direct path), rl (training), or rl_test (testing)'
    )
    
    # World file argument (optional override, defaults to auto-select based on control_mode)
    world_sdf_file_arg = DeclareLaunchArgument(
        'world_sdf_file',
        default_value='auto',  # 'auto' means select based on control_mode
        description='World SDF file path. Use "auto" to auto-select based on control_mode, or specify full path to override'
    )
    
    # Auto-select world file based on control_mode:
    # - rl and rl_test: use world.sdf (no waypoints)
    # - manual, autonomous, diagonal: use world_with_waypoints.sdf (with waypoints)
    world_sdf_file_auto = PythonExpression([
        "'/app/world.sdf' if '", LaunchConfiguration('control_mode'), "' in ['rl', 'rl_test'] else '/app/world_with_waypoints.sdf'"
    ])
    
    # Allow manual override of world file
    world_sdf_file_override = LaunchConfiguration('world_sdf_file')
    
    # --- 2. Launch Gazebo Sim with proper command line flags ---
    # Use override if provided (and not 'auto'), otherwise use auto-selected world file
    final_world_file = PythonExpression([
        "'", world_sdf_file_override, "' if '", world_sdf_file_override, "' != 'auto' else '", world_sdf_file_auto, "'"
    ])
    
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': [final_world_file, ' -r --gui-config ./debugger_config.config']
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
    
    # --- Control Nodes (Conditional based on control_mode) ---
    # Manual control (keyboard)
    manual_pilot = Node(
        package='remote_server_nodes',
        executable='quadcopter_pilot',
        name='quadcopter_pilot',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('control_mode'), "' == 'manual'"
            ])
        )
    )
    
    # Autonomous control (waypoint/L-shaped path)
    autonomous_pilot = Node(
        package='remote_server_nodes',
        executable='autonomous_pilot',
        name='autonomous_pilot',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('control_mode'), "' == 'autonomous'"
            ])
        )
    )
    
    # Diagonal pilot (direct path)
    diagonal_pilot = Node(
        package='remote_server_nodes',
        executable='diagonal_pilot',
        name='diagonal_pilot',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('control_mode'), "' == 'diagonal'"
            ])
        )
    )
    
    # RL training pilot
    rl_pilot = Node(
        package='remote_server_nodes',
        executable='rl_pilot',
        name='rl_pilot',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('control_mode'), "' == 'rl'"
            ])
        )
    )
    
    # RL test pilot
    rl_test_pilot = Node(
        package='remote_server_nodes',
        executable='rl_test',
        name='rl_test',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('control_mode'), "' == 'rl_test'"
            ])
        )
    )
    
    # Camera follower (optional)
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
        control_mode_arg,
        world_sdf_file_arg,
        gazebo_sim,
        cmd_vel_bridge,
        pose_bridge,
        rsrp_bridge,
        cell_id_bridge,
        ns3_node,
        manual_pilot,
        autonomous_pilot,
        diagonal_pilot,
        rl_pilot,
        rl_test_pilot,
        #camera_follower  # Uncomment if you want camera following
    ])
