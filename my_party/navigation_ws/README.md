# Scout Navigation Workspace

This workspace contains custom Nav2 plugins for the Scout Mini.

`scout_navigation_plugins` currently provides the plugin skeleton for
`ScoutAstarPlanner`. It must not be enabled in Nav2 until the A* search and
costmap collision checks are implemented.

Build after Nav2 is installed:

```bash
source /opt/ros/humble/setup.bash
cd /workspaces/ROS2_FOR_SCOUT_MINI/my_party/navigation_ws
colcon build --packages-select scout_navigation_plugins
```
