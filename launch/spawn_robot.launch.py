import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    name_1 = "robot_1"
    name_2 = "robot_2"

    pkg_multi_robot_navigation = get_package_share_directory('multi_robot_navigation')

    gazebo_models_path, ignore_last_dir = os.path.split(pkg_multi_robot_navigation)
    os.environ["GZ_SIM_RESOURCE_PATH"] += os.pathsep + gazebo_models_path

    world_arg = DeclareLaunchArgument(
        'world', default_value='home.sdf',
        description='Name of the Gazebo world file to load'
    )

    model_arg = DeclareLaunchArgument(
        'model', default_value='mogi_bot.urdf',
        description='Name of the URDF description to load'
    )

    x_arg = DeclareLaunchArgument(
        'x', default_value='2.5',
        description='x coordinate of spawned robot'
    )

    y_arg = DeclareLaunchArgument(
        'y', default_value='1.5',
        description='y coordinate of spawned robot'
    )

    yaw_arg = DeclareLaunchArgument(
        'yaw', default_value='-1.5707',
        description='yaw angle of spawned robot'
    )

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Flag to enable use_sim_time'
    )

    xacro_file = os.path.join(pkg_multi_robot_navigation,
                              "urdf",
                              f"mogi_bot.urdf")

    robot_description_content_1 = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            xacro_file,
            " ",
            "name:=",
            "mogi_bot_1",
            " ",
            "prefix:=",
            name_1
        ]
    )

    robot_description_content_2 = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            xacro_file,
            " ",
            "name:=",
            "mogi_bot_2",
            " ",
            "prefix:=",
            name_2
        ]
    )

    gz_bridge_params_path = os.path.join(
        get_package_share_directory('multi_robot_navigation'),
        'config',
        'gz_bridge.yaml'
    )

    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_multi_robot_navigation, 'launch', 'world.launch.py'),
        ),
        launch_arguments={
        'world': LaunchConfiguration('world'),
        }.items()
    )

    # Spawn the URDF model using the `/world/<world_name>/create` service
    spawn_urdf_node_1 = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-entity", name_1,
            "-string", robot_description_content_1,
            "-robot_namespace", name_1,
            "-x", LaunchConfiguration('x'), "-y", LaunchConfiguration('y'), "-z", "0.5", "-Y", LaunchConfiguration('yaw')  # Initial spawn position
        ],
        output="screen",
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    robot_state_publisher_node_1 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=name_1,
        output='screen',
        parameters=[
            {'frame_prefix': name_1 + '/',
             'robot_description': robot_description_content_1,
             'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    spawn_urdf_node_2 = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-entity", name_2,
            "-string", robot_description_content_2,
            "-robot_namespace", name_2,
            "-x", "3.5", "-y", "1", "-z", "0.5", "-Y", LaunchConfiguration('yaw')  # Initial spawn position
        ],
        output="screen",
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    robot_state_publisher_node_2 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=name_2,
        output='screen',
        parameters=[
            {'frame_prefix': name_2 + '/',
             'robot_description': robot_description_content_2,
             'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    # Node to bridge /cmd_vel and /odom
    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args', '-p',
            f'config_file:={gz_bridge_params_path}'
        ],
        output="screen",
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    # Node to bridge /cmd_vel and /odom
    gz_image_bridge_node = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=[
            "/robot_1/camera/image",
            "/robot_2/camera/image",
        ],
        output="screen",
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time'),
             'robot_1.camera.image.compressed.jpeg_quality': 75,
             'robot_2.camera.image.compressed.jpeg_quality': 75},
        ],
    )

    # Relay node to republish camera_info to /camera_info
    relay_camera_info_node_1 = Node(
        package='topic_tools',
        executable='relay',
        name='relay_camera_info',
        namespace=name_1,
        output='screen',
        arguments=['camera/camera_info', 'camera/image/camera_info'],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    # Relay node to republish camera_info to /camera_info
    relay_camera_info_node_2 = Node(
        package='topic_tools',
        executable='relay',
        name='relay_camera_info',
        namespace=name_2,
        output='screen',
        arguments=['camera/camera_info', 'camera/image/camera_info'],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    trajectory_node_1 = Node(
        package='mogi_trajectory_server',
        executable='mogi_trajectory_server',
        name='mogi_trajectory_server_1',
        namespace=name_1,
        parameters=[{'reference_frame_id': 'world',
                     'robot_frame_id': 'robot_1/base_footprint',
                     'trajectory_topic': '/robot_1/trajectory'}]
    )

    trajectory_node_2 = Node(
        package='mogi_trajectory_server',
        executable='mogi_trajectory_server',
        name='mogi_trajectory_server_2',
        namespace=name_2,
        parameters=[{'reference_frame_id': 'world',
                     'robot_frame_id': 'robot_2/base_footprint',
                     'trajectory_topic': '/robot_2/trajectory'}]
    )

    ekf_node_1 = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        namespace=name_1,
        output='screen',
        parameters=[
            os.path.join(pkg_multi_robot_navigation, 'config', 'ekf.yaml'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
             ]
    )

    ekf_node_2 = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        namespace=name_2,
        parameters=[
            os.path.join(pkg_multi_robot_navigation, 'config', 'ekf.yaml'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
             ]
    )

    launchDescriptionObject = LaunchDescription()

    launchDescriptionObject.add_action(world_arg)
    launchDescriptionObject.add_action(model_arg)
    launchDescriptionObject.add_action(x_arg)
    launchDescriptionObject.add_action(y_arg)
    launchDescriptionObject.add_action(yaw_arg)
    launchDescriptionObject.add_action(sim_time_arg)
    launchDescriptionObject.add_action(world_launch)
    launchDescriptionObject.add_action(spawn_urdf_node_1)
    launchDescriptionObject.add_action(robot_state_publisher_node_1)
    launchDescriptionObject.add_action(spawn_urdf_node_2)
    launchDescriptionObject.add_action(robot_state_publisher_node_2)
    launchDescriptionObject.add_action(gz_bridge_node)
    launchDescriptionObject.add_action(gz_image_bridge_node)
    launchDescriptionObject.add_action(relay_camera_info_node_1)
    launchDescriptionObject.add_action(relay_camera_info_node_2)
    launchDescriptionObject.add_action(trajectory_node_1)
    launchDescriptionObject.add_action(trajectory_node_2)
    launchDescriptionObject.add_action(ekf_node_1)
    launchDescriptionObject.add_action(ekf_node_2)

    return launchDescriptionObject
