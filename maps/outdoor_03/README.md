# outdoor_03

导航参数从 outdoor_02 复制。路线已配置为按 1--22 依次运行。

三维地图和二维 Nav2 地图已于 2026-08-19 保存。机械臂观察位、UV 放置以后再采，栈启动方式与现在相同。

## 目录里现在有什么

| 文件 | 状态 |
| --- | --- |
| `nav2_params.yaml` | 已拷自 outdoor_02（速度、膨胀、MPPI、行为树） |
| `initial_pose.yaml` | 已放，起步暂定原点 |
| `recorded_poses.yaml` | 已写入完整 1--22 路线坐标 |
| `map.pcd` | 已保存 |
| `nav2_map.yaml` / `nav2_map.png` | 已生成 |
| `poses.txt` / `patches/` | 已保存 |

## 已记录坐标

坐标在 `map` 系，位置为 `base_link` 中心；航向 `yaw` 单位为弧度。

| 编号 | 描述 | x | y | yaw |
| --- | --- | ---: | ---: | ---: |
| 1 | 红绿灯1 | 10.523 | 1.003 | -0.067 |
| 2 | 红绿灯2 | 24.241 | 0.710 | 0.013 |
| 3 | 拐弯1 | 41.490 | 0.280 | 0.006 |
| 4 | 拐弯2 | 41.606 | -26.727 | -1.587 |
| 5 | 抓取预停点 | 36.966 | -28.583 | 3.127 |
| 6 | 抓取点 | 34.905 | -28.586 | -3.140 |
| 7 | 越过锥桶1 | 27.342 | -30.667 | 3.124 |
| 8 | 越过锥桶2 | 22.361 | -28.384 | 3.135 |
| 9 | 放置预停点 | 16.910 | -28.611 | -3.118 |
| 10 | 放置点 | 15.689 | -28.699 | -3.134 |
| 11 | 返程1 | 11.653 | -28.658 | 3.116 |
| 12 | 返程2 | 21.260 | -31.608 | 0.009 |
| 13 | 返程3 | 44.789 | -31.437 | -0.005 |
| 14 | 返程4 | 41.993 | -28.879 | -3.092 |
| 15 | 抓取预停点 | 36.966 | -28.583 | 3.127 |
| 16 | 抓取点 | 34.905 | -28.586 | -3.140 |
| 17 | 越过锥桶1 | 27.342 | -30.667 | 3.124 |
| 18 | 越过锥桶2 | 22.361 | -28.384 | 3.135 |
| 19 | 放置预停点 | 16.910 | -28.611 | -3.118 |
| 20 | 放置点 | 15.689 | -28.699 | -3.134 |
| 21 | 即将避障 | 3.214 | -28.970 | -3.112 |
| 22 | 终点 | 1.342 | -5.903 | 1.593 |

路线文件已使用以上 1--22 全部坐标。

## 路线精度与启动标志

普通点以连续导航通过：1--4、7--8、11--14、17--18、21--22。它们不会为了旧记录的朝向在目标点附近反复修正。

5、9、15、19 是高精度预停点；其后对应的 6、10、16、20 是抓取/放置高精度点。前往每个预停点时自动降速；到预停点后，会复用已有的精细停车：先正常导航到预停点，再以低速、里程计锁定的方式前进至抓取/放置点，同时保留车头正前方窄区域的紧急停车保护。

三个标志位：

| 标志位 | `0` | `1` |
| --- | --- | --- |
| `SKIP_OUTBOUND` | 正常运行 1--22 | 跳过 1--4，从 5 开始 |
| `RED_FLAG_START_ENABLED` | 不等红旗，直接开始 | 等红旗信号后开始 |
| `ARM_HANDOFF_ENABLED` | 仅导航，到达 6/10/16/20 不交给机械臂 | 到达抓取/放置点后交给机械臂 |

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
ros2 launch scout_navigation_bringup navigation_system.launch.py \
  map_dir:=~/auto/ROS2_FOR_SCOUT_MINI/maps/outdoor_03 can_port:=can2

# 跳过去程时导航也要 SKIP_OUTBOUND=1，才会读 skip_outbound_initial_pose.yaml
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

跳过去程 1--4、直接从抓取预停点 5 开始：

```bash
source ~/auto/ROS2_FOR_SCOUT_MINI/setup_local.bash
SKIP_OUTBOUND=1 RED_FLAG_START_ENABLED=0 ARM_HANDOFF_ENABLED=0 \
ros2 run scout_navigation_bringup run_outdoor03_recorded_route.sh
```
