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
| 4 | 抓取点 | 35.175 | -28.397 | -3.122 |
| 5 | 越过锥桶1 | 27.00 | -30.23 | -3.138 |
| 6 | 越过锥桶2 | 24.71 | -28.43 | -3.087 |
| 7 | 放置预停点 | 17.05 | -28.52 | -3.109 |
| 8 | 放置点 | 15.92 | -28.58 | 3.139 |
| 9 | 返程点1 | 20.99 | -31.41 | 0.057 |
| 10 | 返程点2 | 41.99 | -31.01 | -0.016 |
| 11 | 返程点3 | 44.74 | -28.29 | -3.121 |
| 12 | 抓取预停点（复用点3） | 36.555 | -28.383 | -3.118 |
| 13 | 抓取点（复用点4） | 35.175 | -28.397 | -3.122 |
| 14 | 越过锥桶1（复用点5） | 27.00 | -30.23 | -3.138 |
| 15 | 越过锥桶2（复用点6） | 24.71 | -28.43 | -3.087 |
| 16 | 放置预停点（复用点7） | 17.05 | -28.52 | -3.109 |
| 17 | 放置点（复用点8） | 15.92 | -28.58 | 3.139 |
| 18 | 准备避障 | 6.01 | -28.84 | -3.085 |
| 19 | 终点 | 1.33 | -5.69 | 1.557 |

## 路线执行顺序

完整路线按 `1 → 2 → 3 → 4 → 5 → 6 → 7 → 8 → 9 → 10 → 11 → 12 → 13 → 14 → 15 → 16 → 17 → 18 → 19` 执行。

- 点 3→4、7→8、12→13、16→17：先使用普通导航到高精度预停点，再低速精细停车到抓取/放置点；其余点均为普通导航点。
- 点 5→6、14→15：锥桶路段按普通导航连续通过，中间不为对准航向而停车。
- 点 4、8、13、17：仅在 `ARM_HANDOFF_ENABLED=1` 时交给机械臂；设为 `0` 时只运行导航。
- `RED_FLAG_START_ENABLED=0` 跳过红旗；`SKIP_OUTBOUND=1` 跳过点 1，从点 2 开始。

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

# 终端 2～4：机械臂（与现在相同）
cd ~/auto/Robot_arm/source
./scripts/run_piper_driver.sh
./scripts/run_piper_moveit_ik.sh
source ./scripts/ros_env_graspnet.sh
./scripts/run_distributed_stack_graspnet.sh --robot-backend ros2 \
  --pose-execution-mode moveit_ik --with-piper-driver --with-moveit-ik --warmup
ros2 service call /grasp_pipeline/probe std_srvs/srv/Trigger "{}"

# 终端 5：完整任务。等红旗，到抓取/放置点后交给机械臂。
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

跳过拐弯1（点 3）、直接从拐弯2（点 4）开始测试导航：

```bash
source ~/auto/ROS2_FOR_SCOUT_MINI/setup_local.bash
SKIP_OUTBOUND=1 RED_FLAG_START_ENABLED=0 ARM_HANDOFF_ENABLED=0 \
ros2 run scout_navigation_bringup run_outdoor03_recorded_route.sh
```

挑点，跳过红旗，开启机械臂
```bash
source ~/auto/ROS2_FOR_SCOUT_MINI/setup_local.bash
SKIP_OUTBOUND=1 RED_FLAG_START_ENABLED=0 ARM_HANDOFF_ENABLED=1 \
ros2 run scout_navigation_bringup run_outdoor03_recorded_route.sh

```
