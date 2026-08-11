# indoor_03 地图

保存时间：2026-08-12。

本目录中的 `map.pcd`、`poses.txt` 和 `patches/` 来自同一次 PGO 保存；
`nav2_map.png` 和 `nav2_map.yaml` 由本次三维地图离线转换生成，并启用了车体
自反射点过滤。

导航参数 `nav2_params.yaml` 与初始位姿 `initial_pose.yaml` 直接复制自
`maps/indoor_02`，作为 indoor_03 的初始导航配置。

## 目标坐标点

坐标均位于 `map` 坐标系，角度单位为弧度。

| 序号 | 备注 | x (m) | y (m) | yaw (rad) |
| --- | --- | ---: | ---: | ---: |
| 1 | 门口（红绿灯） | 3.711 | -0.579 | -0.068 |
| 2 | 取件预停点 | 10.455 | -5.603 | -0.092 |
| 3 | 取件（抓取点） | 11.292 | -5.682 | -0.094 |
| 4 | 另一侧1 | 13.065 | -12.413 | 3.092 |
| 5 | 放置预停点 | 10.877 | -5.582 | -0.122 |
| 6 | 放置点 | 11.472 | -5.684 | -0.115 |
| 7 | 另一侧2 | 13.065 | -12.413 | 3.092 |
| 8 | 终点（起点反方向） | 3.711 | -0.579 | 3.074 |

## 顺序路线

执行顺序为：门口（红绿灯）→ 取件预停点 → 取件 → 另一侧1 →
放置预停点 → 放置点 → 另一侧2 → 终点（起点反方向）。

- 门口、另一侧1、另一侧2、终点使用普通导航精度。
- 取件预停点和放置预停点使用精确 Nav2 导航，位置与朝向容差均为
  `0.08`。
- 预停点到取件点、放置点不再使用普通路径规划，而是保持记录朝向，以
  `0.12 m/s` 上限低速靠近；前方 `0.35 m` 的窄区域仍保留雷达紧急停止。
- 任一精确阶段未成功时，路线会停止，不会跳过预停点直接执行低速靠近。
- 全部导航阶段由同一个常驻 ROS 节点连续调度。到达一个点后立即发送下一
  阶段，不再因反复启动 `ros2 run` 节点而停顿 4～8 秒；机械臂实际抓取、
  放置期间仍会按安全流程停车。

启动完整路线：

```bash
source ~/auto/ROS2_FOR_SCOUT_MINI/setup_local.bash
ros2 run scout_navigation_bringup run_indoor03_recorded_route.sh
```

启动导航：

```bash
source ~/auto/ROS2_FOR_SCOUT_MINI/setup_local.bash

ros2 launch scout_navigation_bringup navigation_system.launch.py \
  map_dir:=~/auto/ROS2_FOR_SCOUT_MINI/maps/indoor_03 \
  can_port:=can2
```
