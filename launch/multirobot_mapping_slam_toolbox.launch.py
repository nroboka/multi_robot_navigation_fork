import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import LifecycleNode
from launch.actions import (DeclareLaunchArgument, EmitEvent, LogInfo,
                            RegisterEventHandler)
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from launch.events import matches_action

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

    declare_slam_params_file_1 = DeclareLaunchArgument(
        'slam_params_file_1',
        default_value=os.path.join(get_package_share_directory("multi_robot_navigation"),
                                   'config', 'slam_toolbox_mapping_1.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

    declare_slam_params_file_2 = DeclareLaunchArgument(
        'slam_params_file_2',
        default_value=os.path.join(get_package_share_directory("multi_robot_navigation"),
                                   'config', 'slam_toolbox_mapping_2.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')


    # Generate path to config file
    interactive_marker_config_file_path = os.path.join(
        get_package_share_directory('interactive_marker_twist_server'),
        'config',
        'linear.yaml'
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
            arguments=['-2.0', '-0.2', '0.0', '0.0', '0.0', '0.0', '1.0', 'world', 'robot_1/map'],
    	    parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}])

    static_world_transform_2 = Node( 
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_2',
            namespace=name_2,
            arguments=['-5.0', '0.8', '0.0', '0.0', '0.0', '0.0', '1.0', 'world', 'robot_2/map'],
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


    start_async_slam_toolbox_node_1 = LifecycleNode(
        parameters=[
          LaunchConfiguration('slam_params_file_1'),
          {
            'use_lifecycle_manager': False,
            'use_sim_time': LaunchConfiguration('use_sim_time')
          }
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        namespace=name_1,
        remappings=[
            ("/map", "map"),
            ("/map_metadata", "map_metadata"),
            ("/slam_toolbox/scan_visualization", "slam_toolbox/scan_visualization"),
            ("/slam_toolbox/graph_visualization", "slam_toolbox/graph_visualization"),
        ]
    )

    configure_event_1 = EmitEvent(
        event=ChangeState(
          lifecycle_node_matcher=matches_action(start_async_slam_toolbox_node_1),
          transition_id=Transition.TRANSITION_CONFIGURE
        )
    )

    activate_event_1 = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=start_async_slam_toolbox_node_1,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                LogInfo(msg="[LifecycleLaunch] Slamtoolbox node is activating."),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(start_async_slam_toolbox_node_1),
                    transition_id=Transition.TRANSITION_ACTIVATE
                ))
            ]
        )
    )

    start_async_slam_toolbox_node_2 = LifecycleNode(
        parameters=[
          LaunchConfiguration('slam_params_file_2'),
          {
            'use_lifecycle_manager': False,
            'use_sim_time': LaunchConfiguration('use_sim_time')
          }
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        namespace=name_2,
        remappings=[
            ("/map", "map"),
            ("/map_metadata", "map_metadata"),
            ("/slam_toolbox/scan_visualization", "slam_toolbox/scan_visualization"),
            ("/slam_toolbox/graph_visualization", "slam_toolbox/graph_visualization"),
        ]
    )

    configure_event_2 = EmitEvent(
        event=ChangeState(
          lifecycle_node_matcher=matches_action(start_async_slam_toolbox_node_2),
          transition_id=Transition.TRANSITION_CONFIGURE
        )
    )

    activate_event_2 = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=start_async_slam_toolbox_node_2,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                LogInfo(msg="[LifecycleLaunch] Slamtoolbox node is activating."),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(start_async_slam_toolbox_node_2),
                    transition_id=Transition.TRANSITION_ACTIVATE
                ))
            ]
        )
    )


    launchDescriptionObject = LaunchDescription()

    launchDescriptionObject.add_action(rviz_launch_arg)
    launchDescriptionObject.add_action(rviz_config_arg)
    launchDescriptionObject.add_action(sim_time_arg)
    launchDescriptionObject.add_action(declare_slam_params_file_1)
    launchDescriptionObject.add_action(declare_slam_params_file_2)
    launchDescriptionObject.add_action(rviz_node)
    launchDescriptionObject.add_action(static_world_transform_1)
    launchDescriptionObject.add_action(static_world_transform_2)
    launchDescriptionObject.add_action(interactive_marker_twist_server_node_1)
    launchDescriptionObject.add_action(interactive_marker_twist_server_node_2)
    launchDescriptionObject.add_action(start_async_slam_toolbox_node_1)
    launchDescriptionObject.add_action(configure_event_1)
    launchDescriptionObject.add_action(activate_event_1)
    launchDescriptionObject.add_action(start_async_slam_toolbox_node_2)
    launchDescriptionObject.add_action(configure_event_2)
    launchDescriptionObject.add_action(activate_event_2)


    return launchDescriptionObject