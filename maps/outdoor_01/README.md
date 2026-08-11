# outdoor_01 点位记录

本文件记录 `outdoor_01` 地图的固定导航点。坐标系为 `map`，`yaw` 单位为弧度。

## 当前固定点位

| 名称 | x | y | yaw | 描述 |
| --- | ---: | ---: | ---: | --- |
| 目标点 1 | 19.940 | 41.160 | 1.614 | 路线第一站，使用普通 Nav2 导航。 |
| 目标点 2 | 20.900 | 16.380 | -2.408 | 路线第二站，使用普通 Nav2 导航。 |
| 预停点 | 1.754 | 0.355 | 3.140 | 最终停车前的精细导航点；车头应与最终停车方向平行。 |
| 最终目标点 3 | 0.126 | 0.394 | 3.113 | 靠近障碍物的精细停车点；从预停点低速直行进入。 |
| 启动位置 | 实时读取 | 实时读取 | 实时读取 | 每次任务启动时自动记录，全部任务结束后返回此处。 |

## 任务顺序

```text
启动时记录当前位置
→ 目标点 1
→ 目标点 2
→ 预停点
→ 低速直行到最终目标点 3
→ 临时向前直行 1.5 m
→ 返回启动位置
```

目标点 1、2 允许正常规划、绕障和重规划。

预停点使用精细导航。到达后，末段控制不采用 Nav2 局部代价地图，而是先将车头对齐到最终目标点朝向，再低速直行。车头前方的窄安全检测仍然保留。

## 当前末段参数

| 参数 | 当前值 | 含义 |
| --- | ---: | --- |
| 末段速度 | 0.14 m/s | 从预停点驶向最终目标点 3 的速度。 |
| 最终位置误差 | 0.10 m | 距离最终点小于该值视为到达。 |
| 最终朝向误差 | 0.12 rad | 最终车头允许误差。 |
| 预对齐误差 | 0.04 rad | 低速前进前必须达到的车头对齐精度。 |
| 最大末段角速度 | 0.16 rad/s | 末段仅允许小幅朝向修正。 |
| 临时脱困前进距离 | 1.50 m | 到达最终目标点 3 后，沿当前停车朝向再向前直行；机械臂接管后关闭。 |

临时脱困前进由 `post_dock_forward_enabled` 控制，当前为启用。机械臂任务接管底盘前，将它改为 `False`，或将 `post_dock_forward_distance` 改为 `0.0`。

## 修改点位

固定点位实际定义在：

`my_party/navigation_ws/src/scout_navigation_bringup/scripts/run_outdoor_recorded_route.py`

对应参数：

```text
point_1_x / point_1_y / point_1_yaw
point_2_x / point_2_y / point_2_yaw
staging_x / staging_y / staging_yaw
point_3_x / point_3_y / point_3_yaw
```

修改后构建导航包：

```bash
cd ~/auto/ROS2_FOR_SCOUT_MINI/my_party/navigation_ws
source /opt/ros/humble/setup.bash
source ~/auto/ROS2_FOR_SCOUT_MINI/setup_local.bash
colcon build --packages-select scout_navigation_bringup --symlink-install
```

## 运行

完整导航启动、CAN、雷达和定位的通用操作见仓库根目录 `README.md`。

导航定位成功后运行固定路线：

```bash
source ~/auto/ROS2_FOR_SCOUT_MINI/setup_local.bash
ros2 run scout_navigation_bringup run_outdoor_recorded_route.py
```
