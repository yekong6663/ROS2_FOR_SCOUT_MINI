# Jetson：预停点与精细停车路线

本文件只说明“预停点 → 最终停车点 → 控制权交接”的任务流程。CAN、雷达、建图、定位和普通导航的通用说明见仓库根目录 `README.md`。

## 你只需要提供的坐标

每个点都提供地图坐标：`x`、`y`、`yaw`（弧度）。

```text
目标点 1：x=?, y=?, yaw=?
目标点 2：x=?, y=?, yaw=?
预停点：x=?, y=?, yaw=?
最终停车点：x=?, y=?, yaw=?
```

我会把它们写入：

`my_party/navigation_ws/src/scout_navigation_bringup/scripts/run_outdoor_recorded_route.py`

路线固定按以下顺序执行：

```text
记录启动位置
目标点 1
目标点 2
预停点
低速直行到最终停车点
回到启动位置
```

## 两类目标点的区别

目标点 1、目标点 2 使用普通导航：允许正常绕障、重新规划和避障。

预停点使用精细导航：位置误差约 10 cm，朝向误差约 0.12 rad。它应位于最终停车点后方、道路仍然安全的位置。

最终停车点不再由 Nav2 局部代价地图决定最后一段路径。车辆先在预停点对齐，再以低速直行进入最终点。当前默认参数：

```text
末段速度：0.14 m/s
末段位置误差：0.10 m
末段朝向误差：0.12 rad
预对齐误差：0.04 rad
最大末段角速度：0.16 rad/s
```

末段忽略局部代价地图的误识别，但保留车头窄区域的紧急停车检测。若障碍物实际位于车头正前方，车辆会停止，不会强行碰撞。

## 控制权规则

任何时间只能有一个节点向 `/cmd_vel_nav` 发速度：

```text
普通导航：Nav2
末段停车：run_outdoor_recorded_route.py
机械臂微调：机械臂控制节点
```

切换前，前一个控制节点必须发布零速度并结束。底盘接收话题 `/cmd_vel` 不需要修改。

## 预停点如何选

预停点应满足：

- 车头方向与最终停车点 `yaw` 一致，且平行于目标障碍物。
- 最终点在车头前方。
- 横向误差尽量小于 5 cm。
- 与最终点距离建议 0.4 至 2.0 m。
- 预停点本身必须是普通 Nav2 可以安全到达的位置。

例如当前任务：

```text
目标点 1：x=19.940, y=41.160, yaw=1.614
目标点 2：x=20.900, y=16.380, yaw=-2.408
预停点：x=1.754, y=0.355, yaw=3.140
最终停车点：x=0.126, y=0.394, yaw=3.113
```

## 修改坐标后如何构建

```bash
cd ~/auto/ROS2_FOR_SCOUT_MINI/my_party/navigation_ws
source /opt/ros/humble/setup.bash
source ~/auto/ROS2_FOR_SCOUT_MINI/setup_local.bash
colcon build --packages-select scout_navigation_bringup --symlink-install
```

## 如何运行任务

先启动导航：

```bash
source ~/auto/ROS2_FOR_SCOUT_MINI/setup_local.bash
ros2 launch scout_navigation_bringup navigation_system.launch.py map_dir:=~/auto/ROS2_FOR_SCOUT_MINI/maps/outdoor_01 can_port:=can2
```

确认定位成功后，在另一个终端运行：

```bash
source ~/auto/ROS2_FOR_SCOUT_MINI/setup_local.bash
ros2 run scout_navigation_bringup run_outdoor_recorded_route.py
```

旧的路线实例必须先按 `Ctrl-C` 停止，再启动新实例，避免两个路线节点同时发送目标。

## 最终点后的机械臂控制

最终停车点成功后，路线节点会继续返回启动位置。若要在最终点把控制权交给机械臂，请先停止路线节点，或后续把“回到启动位置”设为 `false`，再由机械臂控制节点向 `/cmd_vel_nav` 发布仅前进或后退的低速速度。
