# outdoor_03 地图

保存时间：2026-08-15。

本目录的 `map.pcd`、`poses.txt` 与 `patches/` 由同一次 PGO 建图保存生成。
`nav2_map.png` 与 `nav2_map.yaml` 由该三维地图投影得到，分辨率为 `0.05 m`。

`nav2_params.yaml` 与 `initial_pose.yaml` 复制自 `maps/outdoor_01`，因此使用
outdoor_01 的室外导航参数与速度配置。初始位姿 `(0, 0, 0, 0)` 用作此地图的
ICP 收敛初始猜测；`nav2_map.before_start_area_repair.png` 是二维地图修复前的备份。为避免车辆
自身采集造成的灰色未知区阻止启动，地图中仅清理了初始车体周围半径 `0.65 m` 的
投影残留，其他道路边界和障碍未修改。

启动导航：

```bash
source ~/auto/ROS2_FOR_SCOUT_MINI/setup_local.bash

ros2 launch scout_navigation_bringup navigation_system.launch.py \
  map_dir:=~/auto/ROS2_FOR_SCOUT_MINI/maps/outdoor_03 \
  can_port:=can2
```
