# multi_robot_navigation
Spawning a swarm of robots and utilize the navigation stack using ROS2 Jazzy and Gazebo Harmonic

# 1. Start the simulation with 2 robots

```bash
ros2 launch multi_robot_navigation spawn_robot.launch.py
```

# 2. Run SLAM on both robots

### Cartographer:
```bash
ros2 launch multi_robot_navigation multirobot_mapping_cartographer.launch.py
```

### OR

### slam_toolbox:
```bash
ros2 launch multi_robot_navigation multirobot_mapping_slam_toolbox.launch.py
```

# 3. Start map merging

```bash
ros2 launch multirobot_map_merge map_merge.launch.py
```