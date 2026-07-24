# Scout Navigation Workspace

This workspace contains the Scout Mini Nav2 bringup, configuration, custom
plugins, and a source overlay for `nav2_lifecycle_manager`.

- `scout_navigation_bringup` starts the static map server and the Nav2
  navigation servers. Its local voxel costmap consumes
  `/fastlio2/body_cloud`.
- `scout_navigation_plugins` contains the `ScoutAstarPlanner` skeleton. It
  must not be enabled until its A* search and collision checks are implemented.
- `nav2_lifecycle_manager` is rebuilt from the ROS Humble 1.1.20 source
  package to avoid the binary package's `diagnostic_updater` ABI mismatch.

Build after Nav2 is installed:

```bash
source /opt/ros/humble/setup.bash
source /workspaces/ROS2_FOR_SCOUT_MINI/third_party/fast_lio2_ws/install/setup.bash
source /workspaces/ROS2_FOR_SCOUT_MINI/third_party/scout_mini_ws/install/setup.bash
source /workspaces/ROS2_FOR_SCOUT_MINI/my_party/map_transformation_ws/install/setup.bash
cd /workspaces/ROS2_FOR_SCOUT_MINI/my_party/navigation_ws
colcon build --symlink-install \
  --packages-select nav2_lifecycle_manager scout_navigation_plugins \
  scout_navigation_bringup \
  --cmake-args -DBUILD_TESTING=OFF
```

After FAST-LIO2 localization reports `valid: true`, start Nav2 with:

```bash
source /opt/ros/humble/setup.bash
source /workspaces/ROS2_FOR_SCOUT_MINI/my_party/navigation_ws/install/setup.bash
ros2 launch scout_navigation_bringup navigation.launch.py \
  map:=/workspaces/ROS2_FOR_SCOUT_MINI/maps/site_01/nav2_map.yaml
```

The preferred entry points are:

```bash
# Mapping nodes; add start_base:=true for ROS teleoperation.
ros2 launch scout_navigation_bringup mapping.launch.py

# Convert map.pcd + poses.txt and save nav2_map.png/yaml in the same folder.
ros2 launch scout_navigation_bringup map_conversion.launch.py \
  map_dir:=/workspaces/ROS2_FOR_SCOUT_MINI/maps/site_01

# Start hardware, localization, automatic initial pose, and gated Nav2.
ros2 launch scout_navigation_bringup navigation_system.launch.py \
  map_dir:=/workspaces/ROS2_FOR_SCOUT_MINI/maps/site_01
```

`navigation_system.launch.py` reads `map.pcd`, `nav2_map.yaml`, and optional
`initial_pose.yaml` from the given directory. Nav2 starts only after the
localizer reports a valid registration.
