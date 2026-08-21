# outdoor_03

导航参数保留，地图与路线坐标已清空，等待重新建图后再配置。

## 目录里现在有什么

| 文件 | 状态 |
| --- | --- |
| `nav2_params.yaml` | 已拷自 outdoor_02（速度、膨胀、MPPI、行为树） |
| `initial_pose.yaml` | 已放，起步暂定原点 |
| `recorded_poses.yaml` | 已清空，等待重新录制 |
| `map.pcd` | 等待重新建图生成 |
| `nav2_map.yaml` / `nav2_map.png` | 等待二维转换生成 |
| `poses.txt` / `patches/` | 等待重新建图生成 |

## 已记录坐标

坐标使用 `map` 系，位置为 `base_link` 中心，航向 `yaw` 单位为弧度。

| 编号 | 描述 | x | y | yaw |
| ---: | --- | ---: | ---: | ---: |
| 1 | 拐弯1 | 39.79 | -0.09 | 0.013 |
| 2 | 拐弯2 | 42.47 | -28.19 | -1.569 |
| 3 | 抓取预停点 | 36.555 | -28.383 | -3.118 |
| 5 | 越过锥桶1 | 27.00 | -30.23 | -3.138 |
| 6 | 越过锥桶2 | 21.174 | -28.228 | -3.125 |
| 7 | 放置预停点 | 17.05 | -28.52 | -3.109 |
| 9 | 返程点1 | 20.99 | -31.41 | 0.057 |
| 10 | 返程点2 | -40.172 | -31.352 | -0.094 |
| 12 | 抓取预停点（复用点3） | 36.555 | -28.383 | -3.118 |
| 14 | 越过锥桶1（复用点5） | 27.00 | -30.23 | -3.138 |
| 15 | 越过锥桶2（复用点6） | 21.174 | -28.228 | -3.125 |
| 16 | 放置预停点（复用点7） | 17.05 | -28.52 | -3.109 |
| 19 | 准备避障 | 3.516 | -26.854 | 1.600 |
| 20 | 终点 | 1.33 | -5.69 | 1.557 |

## 路线执行顺序

完整路线按 `1 → 2 → 3 → 5 → 6 → 7 → 9 → 10 → 12 → 14 → 15 → 16 → 19 → 20` 执行。

- 抓取/放置点 4、8、13、17 已删除；预停点按普通低精度导航通过。
- 点 19 是新的准备避障观察点：到达后停车 3 秒、清理代价地图，再以 `0.30 m/s` 重新规划到终点 20。
- `RED_FLAG_START_ENABLED=0` 跳过红旗；`SKIP_OUTBOUND=1` 从点 2 开始。
- `SKIP_TO_PREPARE=1` 使用点 19 的初始位姿，跳过前面路线，直接执行点 19 的观察和终点流程。

## 后续操作

当前地图已经保存并完成二维转换。若以后重新保存三维地图，需要重新生成二维图：

```bash
source ~/auto/ROS2_FOR_SCOUT_MINI/setup_local.bash
ros2 launch scout_navigation_bringup map_conversion.launch.py \
  map_dir:=~/auto/ROS2_FOR_SCOUT_MINI/maps/outdoor_03
```

1. 核对 `nav2_map.yaml` 的 `origin` / `resolution`，灰色当边界时 `free_thresh` 可参考 outdoor_02 的 `0.01`。
2. 重定位后录每个路点的 `x y yaw`，填进 `recorded_poses.yaml`。未填完时路线脚本会拒绝开跑。
3. 机械臂数据另采（观察位、放置 UV）。桌和盒子相对车的摆法若与 outdoor_02 相同，可先继续用现有示教。

## 启动顺序

先导航，定位好了再开机械臂，最后开路线。不要机械臂已经开着时再重启导航。

```bash
# 遥控器 SWB 最上
source ~/auto/ROS2_FOR_SCOUT_MINI/setup_local.bash
ros2 topic echo /scout_status --once | grep -E 'vehicle_state|control_mode|error_code'

# 终端 1：导航（走去程用默认 initial_pose.yaml）
source ~/auto/ROS2_FOR_SCOUT_MINI/setup_local.bash
ros2 launch scout_navigation_bringup navigation_system.launch.py \
  map_dir:=~/auto/ROS2_FOR_SCOUT_MINI/maps/outdoor_03 can_port:=can2

# 跳过去程时导航也要 SKIP_OUTBOUND=1，才会读 skip_outbound_initial_pose.yaml
source ~/auto/ROS2_FOR_SCOUT_MINI/setup_local.bash
SKIP_OUTBOUND=1 ros2 launch scout_navigation_bringup navigation_system.launch.py \
  map_dir:=~/auto/ROS2_FOR_SCOUT_MINI/maps/outdoor_03 can_port:=can2

# 直接从新的准备避障点19开始：导航使用 skip_prepare_obstacle_initial_pose.yaml
source ~/auto/ROS2_FOR_SCOUT_MINI/setup_local.bash
SKIP_TO_PREPARE=1 ros2 launch scout_navigation_bringup navigation_system.launch.py \
  map_dir:=~/auto/ROS2_FOR_SCOUT_MINI/maps/outdoor_03 can_port:=can2


# 终端 5：完整路线（当前仅导航，不再执行抓取/放置交接）。
source ~/auto/ROS2_FOR_SCOUT_MINI/setup_local.bash
SKIP_OUTBOUND=0 RED_FLAG_START_ENABLED=1 ARM_HANDOFF_ENABLED=1 \
ros2 run scout_navigation_bringup run_outdoor03_recorded_route.sh
```

跳过红旗但保留机械臂接管：

```bash
source ~/auto/ROS2_FOR_SCOUT_MINI/setup_local.bash
SKIP_OUTBOUND=0 RED_FLAG_START_ENABLED=0 ARM_HANDOFF_ENABLED=1 \
ros2 run scout_navigation_bringup run_outdoor03_recorded_route.sh
```

跳过红旗和机械臂接管，仅测试导航：

```bash
source ~/auto/ROS2_FOR_SCOUT_MINI/setup_local.bash
SKIP_OUTBOUND=0 RED_FLAG_START_ENABLED=0 ARM_HANDOFF_ENABLED=0 \
ros2 run scout_navigation_bringup run_outdoor03_recorded_route.sh
```

直接从新的准备避障点19开始测试：

```bash
source ~/auto/ROS2_FOR_SCOUT_MINI/setup_local.bash
SKIP_TO_PREPARE=1 RED_FLAG_START_ENABLED=0 ARM_HANDOFF_ENABLED=0 \
ros2 run scout_navigation_bringup run_outdoor03_recorded_route.sh

```

挑点，跳过红旗，开启机械臂
```bash
source ~/auto/ROS2_FOR_SCOUT_MINI/setup_local.bash
SKIP_OUTBOUND=1 RED_FLAG_START_ENABLED=0 ARM_HANDOFF_ENABLED=1 \
ros2 run scout_navigation_bringup run_outdoor03_recorded_route.sh

```

全部
```bash
source ~/auto/ROS2_FOR_SCOUT_MINI/setup_local.bash
SKIP_OUTBOUND=0 RED_FLAG_START_ENABLED=1 ARM_HANDOFF_ENABLED=1 \
ros2 run scout_navigation_bringup run_outdoor03_recorded_route.sh

```

当前 outdoor_03 路线所有目标点均为低精度坐标目标：只要求到达 x/y 范围，不要求记录的朝向 yaw；预停点也不再使用高精度停车。

无红旗、无机械臂接管、不跳过任何路线点（正常启动，不延迟）：

```bash
source ~/auto/ROS2_FOR_SCOUT_MINI/setup_local.bash
SKIP_OUTBOUND=0 RED_FLAG_START_ENABLED=0 ARM_HANDOFF_ENABLED=0 \
ros2 run scout_navigation_bringup run_outdoor03_recorded_route.sh
```

如需完整地图测试时在发布第一个目标点前延迟 30 秒（不是延迟启动导航）：

```bash
source ~/auto/ROS2_FOR_SCOUT_MINI/setup_local.bash
SKIP_OUTBOUND=0 RED_FLAG_START_ENABLED=0 ARM_HANDOFF_ENABLED=0 \
OUTDOOR03_STARTUP_DELAY_SEC=30 \
ros2 run scout_navigation_bringup run_outdoor03_recorded_route.sh
```
