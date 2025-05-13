import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    name_1 = "robot_1"
    name_2 = "robot_2"

    pkg_multi_robot_navigation = get_package_share_directory('multi_robot_navigation')

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Open RViz'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value='rviz.rviz',
        description='RViz config file'
    )

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Flag to enable use_sim_time'
    )

    # Generate path to config file
    interactive_marker_config_file_path = os.path.join(
        get_package_share_directory('interactive_marker_twist_server'),
        'config',
        'linear.yaml'
    )

    cartographer_config_dir = PathJoinSubstitution(
        [
            FindPackageShare('multi_robot_navigation'),
            'config',
        ]
    )

    # Launch rviz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg_multi_robot_navigation, 'rviz', LaunchConfiguration('rviz_config')])],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    static_world_transform_1 = Node( 
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_1',
            namespace=name_1,
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', '1.0', 'world', 'robot_1/map'],
    	    parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}])

    static_world_transform_2 = Node( 
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_2',
            namespace=name_2,
            arguments=['0.5', '1.0', '0.0', '0.0', '0.0', '0.0', '1.0', 'world', 'robot_2/map'],
    	    parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}])

    interactive_marker_twist_server_node_1 = Node(
            package='interactive_marker_twist_server',
            executable='marker_server',
            namespace=name_1,
            parameters=[{'link_name': 'robot_1/base_link'}],
            remappings=[('/cmd_vel', '/robot_1/cmd_vel')])

    interactive_marker_twist_server_node_2 = Node(
            package='interactive_marker_twist_server',
            executable='marker_server',
            namespace=name_2,
            parameters=[{'link_name': 'robot_2/base_link'}],
            remappings=[('/cmd_vel', '/robot_2/cmd_vel')])

    cartographer_1 = Node(
            package='cartographer_ros',
            executable='cartographer_node',
            output='screen',
            namespace=name_1,
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', "cartographer_1.lua"])

    cartographer_occupancy_1 = Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            output='screen',
            namespace=name_1,
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            arguments=['-resolution', '0.05'])

    cartographer_2 = Node(
            package='cartographer_ros',
            executable='cartographer_node',
            output='screen',
            namespace=name_2,
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', "cartographer_2.lua"])

    cartographer_occupancy_2 = Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            output='screen',
            namespace=name_2,
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            arguments=['-resolution', '0.05'])

    launchDescriptionObject = LaunchDescription()

    launchDescriptionObject.add_action(rviz_launch_arg)
    launchDescriptionObject.add_action(rviz_config_arg)
    launchDescriptionObject.add_action(sim_time_arg)
    launchDescriptionObject.add_action(rviz_node)
    launchDescriptionObject.add_action(static_world_transform_1)
    launchDescriptionObject.add_action(static_world_transform_2)
    launchDescriptionObject.add_action(interactive_marker_twist_server_node_1)
    launchDescriptionObject.add_action(interactive_marker_twist_server_node_2)
    launchDescriptionObject.add_action(cartographer_1)
    launchDescriptionObject.add_action(cartographer_occupancy_1)
    launchDescriptionObject.add_action(cartographer_2)
    launchDescriptionObject.add_action(cartographer_occupancy_2)

    return launchDescriptionObject