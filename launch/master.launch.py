import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    
    pkg_name = 'rover_control'
    pkg_share = get_package_share_directory(pkg_name)

    # Paths
    urdf_file_path = os.path.join(pkg_share, 'urdf', 'rover.urdf')
    controllers_file_path = os.path.join(pkg_share, 'config', 'my_controllers.yaml')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'robot.rviz')
    
    # Process URDF
    doc = xacro.process_file(urdf_file_path, mappings={'controllers_yaml': controllers_file_path})
    robot_description = {'robot_description': doc.toxml()}

    # 1. Gazebo Simulation
    launch_gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # 2. Spawn Robot
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'my_rover', '-topic', 'robot_description', '-z', '0.5'],
        output='screen'
    )

    # 3. Robot State Publisher
    node_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # 4. Bridge (Gazebo <-> ROS)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen'
    )

    # 5. Controllers
    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    rover_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["rover_controller"],
    )

    # 6. YOUR CUSTOM BRIDGE (Python Script)
    teleop_bridge_node = Node(
        package='rover_control',
        executable='rover_teleop_bridge', # Uses the entry point from setup.py
        output='screen'
    )

    # 7. TELEOP KEYBOARD (Pop-up Window)
    # We delay this slightly to make sure everything else loads first
    teleop_keyboard_node = TimerAction(
        period=5.0, 
        actions=[
            Node(
                package='teleop_twist_keyboard',
                executable='teleop_twist_keyboard',
                name='teleop_twist_keyboard',
                output='screen',
                prefix='xterm -e', # Opens in a new terminal window
                parameters=[{'use_sim_time': True}]
            )
        ]
    )

    # 8. RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        launch_gz,
        bridge,
        node_rsp,
        spawn_entity,
        teleop_bridge_node,
        rviz_node,
        
        # Controller Flow (Wait for spawn -> Start JSB -> Start Rover Controller -> Start Keyboard)
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[jsb_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=jsb_spawner,
                on_exit=[rover_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=rover_spawner,
                on_exit=[teleop_keyboard_node],
            )
        ),
    ])