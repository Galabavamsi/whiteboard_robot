import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    pkg_path = get_package_share_directory('whiteboard_robot')
    world_path = os.path.join(pkg_path, 'world', 'whiteboard.world')
    urdf_file_path = os.path.join(pkg_path, 'urdf', 'gantry.urdf.xacro')
    robot_description_raw = xacro.process_file(urdf_file_path).toxml()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': '-r ' + world_path}.items(),
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw, 'use_sim_time': True}] 
    )

    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description', '-entity', 'whiteboard_robot'],
                        output='screen')

    joint_state_broadcaster_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    x_controller_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["x_position_controller", "--controller-manager", "/controller_manager"],
    )

    y_controller_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["y_position_controller", "--controller-manager", "/controller_manager"],
    )

    duster_controller_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["duster_velocity_controller", "--controller-manager", "/controller_manager"],
    )

    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        joint_state_broadcaster_spawner,
        x_controller_spawner,
        y_controller_spawner,
        duster_controller_spawner
    ])
