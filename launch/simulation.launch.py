import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    # 1. SETUP VARIABLES
    pkg_name = 'rover_control'
    pkg_share = get_package_share_directory(pkg_name)
    
    # Paths
    urdf_file_path = os.path.join(pkg_share, 'urdf', 'rover.urdf')
    controllers_file_path = os.path.join(pkg_share, 'config', 'my_controllers.yaml')
    
    # Process URDF with xacro (even if it's .urdf) to resolve paths
    doc = xacro.process_file(urdf_file_path, mappings={'controllers_yaml': controllers_file_path})
    robot_description = {'robot_description': doc.toxml()}

    # 2. NODES

    # Robot State Publisher (Publishes TF)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # Gazebo Simulation
    launch_gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # Spawn Entity (The Rover)
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'my_rover',
            '-topic', 'robot_description',
            '-x', '0', '-y', '0', '-z', '0.5'
        ],
        output='screen'
    )

    # Bridge (Gazebo Clock -> ROS) - VERY IMPORTANT
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen'
    )

    # 3. CONTROLLERS (Launch them after spawning)
    
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    rover_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["rover_controller"],
    )

    return LaunchDescription([
        launch_gz,
        bridge,
        node_robot_state_publisher,
        spawn_entity,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster,
                on_exit=[rover_controller],
            )
        ),
    ])

