# 工作空间结构

```
ROS2_FOR_SCOUT_MINI/
├── third_party/                          # 第三方依赖（只编译一次）
│   ├── fast_lio2_ws/                     # FAST-LIO2 相关工作空间
│   │   └── src/
│   │       ├── FASTLIO2_ROS2/            # FAST-LIO2 完整版（里程计+回环+重定位+地图优化）
│   │       ├── livox_ros_driver2/        # Livox ROS2 雷达驱动
│   │       ├── Livox-SDK2/               # Livox 底层 SDK（cmake 安装）
│   │       └── Sophus/                   # 李群/李代数库（cmake 安装）
│   └── scout_mini_ws/                    # Scout Mini 相关工作空间
│       └── src/
│           ├── scout_ros2/               # Scout Mini 底盘驱动（多功能包子目录）
│           └── ugv_sdk/                  # 底盘通信 SDK
├── mine_ws/                              # 你自己的 ROS2 包（频繁修改）
│   └── src/
├── log/
└── 比赛规则.pdf
```

> **设计思路**：三个独立工作空间。第三方包按功能拆分为 `fast_lio2_ws/` 和 `scout_mini_ws/`，只编译一次；你自己的包在 `mine_ws/`，修改后单独编译。通过 overlay 叠加使用：
> ```bash
> source third_party/fast_lio2_ws/install/setup.bash    # 先 source FAST-LIO2 依赖
> source third_party/scout_mini_ws/install/setup.bash   # 再 source Scout 底盘依赖
> source mine_ws/install/setup.bash                     # 最后 source 自己的
> ```


# SCOUT MINI底盘


## 1. 基本代码
官方仓库[链接](https://github.com/agilexrobotics/scout_ros2)

```bash
# 克隆到底盘工作空间
cd /workspaces/ROS2_FOR_SCOUT_MINI/third_party/scout_mini_ws/src
git clone https://github.com/westonrobot/ugv_sdk.git --depth 1
git clone https://github.com/westonrobot/scout_ros2.git --depth 1

# 编译底盘工作空间
cd /workspaces/ROS2_FOR_SCOUT_MINI/third_party/scout_mini_ws
colcon build 
```

### 1.2 补充安装
#### ASIO库
一个跨平台的 C++ 网络库，用于实现高效的异步通信。它可以处理网络、串口等 I/O 操作，而不会阻塞程序的其他任务。
```bash
sudo apt update
sudo apt install -y libasio-dev
```
docker容器中下载慢一定要配置代理，以及根据代理进行配置
```bash
sudo bash -c 'cat > /etc/apt/apt.conf.d/99proxy <<EOF
Acquire::http::Proxy "http://127.0.0.1:7897";
Acquire::https::Proxy "http://127.0.0.1:7897";
EOF'
```
还有git的
```bash
git config --global http.proxy http://127.0.0.1:7897
git config --global https.proxy http://127.0.0.1:7897
```

## 2. CAN总线配置通讯

> **说明：`modprobe` 加载内核模块必须在宿主机（Ubuntu）中执行，容器共享宿主机内核，无法加载模块。**
> 其余步骤（配置 CAN 接口、测试通信、ROS2 驱动）均可在 Ubuntu 容器内完成，无需切换环境。
>
> | 步骤 | 执行位置 | 原因 |
> |------|---------|------|
> | `modprobe gs_usb` | **Ubuntu 宿主机（仅一次）** | 容器共享宿主机内核，无法加载模块 |
> | `ip link set can0` / `candump` | 容器内 | 已配置 `--cap-add=NET_ADMIN` + `--network=host` |
> | ROS2 驱动（scout_base） | 容器内 | 同容器环境 |

#### 硬件连接
使用 USB-CAN 适配器（如 CANable/周立功等）将工控机连接至 Scout Mini 底盘的 CAN 接口。底盘 CAN 接口定义参考官方手册。

**Scout Mini 底盘 CAN 默认参数：**
- 波特率：500K bps（部分版本为 250K，以实际手册为准）

#### 1. 安装 CAN 工具（容器内执行）
```bash
sudo apt update
sudo apt install -y can-utils
```

#### 2. 使能 gs_usb 内核模块（Ubuntu 宿主机中执行，仅此一步需要在宿主机操作）

```bash
sudo modprobe gs_usb
```

> 如果 `modprobe: FATAL: Module gs_usb not found`，说明当前内核未包含 gs_usb 驱动，需要自行编译内核或安装对应的内核模块包：
> ```bash
> sudo apt install linux-modules-extra-$(uname -r)
> ```
> 安装后再次执行 `sudo modprobe gs_usb`。

输入后应该不会有输出，可以使用以下代码查看
```bash
lsmod | grep gs_usb
```
会输出：
```bash
gs_usb                 24576  0
can_dev                49152  1 gs_usb
```
#### 3. 设置 500K 波特率并使能 CAN-to-USB 适配器（容器内执行）
```bash
sudo apt update && sudo apt install -y iproute2  # 安装iproute2
sudo ip link set can0 up type can bitrate 500000
```

#### 4. 验证 CAN 设备（容器内执行）
```bash
ifconfig -a
```
正常情况下应能看到 `can0` 设备，比如
```bash
can0: flags=193<UP,RUNNING,NOARP>  mtu 16
        unspec 00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00  txqueuelen 10  (UNSPEC)
        RX packets 0  bytes 0 (0.0 B)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 0  bytes 0 (0.0 B)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0
```

#### 5. 测试 CAN 通信（容器内执行）

若此时 CAN-to-USB 已经与 SCOUT MINI 2.0 机器人相连，且小车已开启，使用 `candump` 即可监听来自底盘的数据：

```bash
candump can0
```
> 收到 CAN 帧即表示通信正常，可继续下一步启动 ROS2 驱动。

#### 6. 启动底盘 ROS2 驱动（Docker 容器内执行）

```bash
# 编译底盘工作空间
cd /workspaces/ROS2_FOR_SCOUT_MINI/third_party/scout_mini_ws
colcon build
source install/setup.bash

# 启动 Scout Mini 底盘驱动
ros2 launch scout_base scout_mini_base.launch.py port_name:=can0
```

成功后终端会打印：
```
Detected protocol: AGX_V1 (或 AGX_V2)
Robot initialized, start running ...
```

此时发布 `/cmd_vel` 即可控制底盘运动：
```bash
ros2 topic pub --rate 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

#### 注意事项
1. 黄线接入H，蓝线接入L
2. 黄蓝线需要剥去绝缘层

## 3. 底盘运行测试
执行
```bash
# 开启底盘运动结点
ros2 launch scout_base scout_base.launch.py is_scout_mini:=true
# 使用键盘控制运行
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
键盘控制运行表

---

# 建图SLAM

构建参考开源库：[FASTLIO2_ROS2](https://github.com/liangheming/FASTLIO2_ROS2)（基于 [FAST-LIO2](https://github.com/hku-mars/FAST_LIO) 的 ROS2 重构版）。

## FASTLIO2_ROS2 功能包架构

| 功能包 | 作用 | 说明 |
|--------|------|------|
| `fastlio2` | 激光惯性里程计 | 核心，输出 odom + 去畸变点云 |
| `pgo` | 回环检测 + 位姿图优化 | 基于 ICP + GTSAM，消除长距离累积漂移 |
| `localizer` | 地图重定位 | 加载 `.pcd` 地图做粗到细两阶段 ICP 全局定位 |
| `hba` | 一致性地图精化 | 基于 HBA/BALM 后端优化地图质量 |
| `interface` | 服务接口定义 | `.srv` 文件 |

### 话题与 TF 数据流

```
/livox/lidar ──▶ fastlio2 ──▶ /fastlio2/lio_odom       （里程计，100Hz）
/livox/imu  ──▶           ──▶ /fastlio2/body_cloud     （去畸变点云，10Hz）
                                │
              ┌─────────────────┼─────────────────┐
              ▼                 ▼                  ▼
            pgo             localizer            hba
       回环检测+优化       全局重定位          地图精化
       ┌─────┴─────┐     ┌─────┴─────┐
       │ map→lidar  │     │ map→lidar  │
       │ TF 广播     │     │ TF 广播     │
       └───────────┘     └───────────┘
```

## 环境依赖

```bash
# 基础依赖
sudo apt update
sudo apt install -y libpcl-dev libeigen3-dev libasio-dev

# GTSAM（位姿图优化，pgo 包依赖）
sudo apt install -y ros-humble-gtsam
```

### Livox-SDK2

```bash
cd /workspaces/ROS2_FOR_SCOUT_MINI/third_party/fast_lio2_ws/src
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2 && mkdir build && cd build
cmake .. 
make -j4
sudo make install
```
使用有build的重新构建需要先删除相关文件夹：
```bash
rm -rf build
```
### livox_ros_driver2

```bash
cd /workspaces/ROS2_FOR_SCOUT_MINI/third_party/fast_lio2_ws/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
```

> `build.sh` 除了编译，还会做三件必要的事：复制 `package_ROS2.xml` → `package.xml`、复制 `launch_ROS2/` → `launch/`、传入 `-DROS_EDITION=ROS2`。所以**首次构建必须执行脚本**。但它内部调用 `colcon build` 会编译 `src/` 下所有包，如果 FASTLIO2_ROS2 已存在会一起编译导致卡死。
>
> **正确做法**：先构建 livox，再 clone FASTLIO2_ROS2：

```bash
# 此时 src/ 下只有 Livox-SDK2 和 livox_ros_driver2，没有 FASTLIO2_ROS2
cd /workspaces/ROS2_FOR_SCOUT_MINI/third_party/fast_lio2_ws/src/livox_ros_driver2
source /opt/ros/humble/setup.sh
./build.sh humble
```

> 如果已经 clone 了 FASTLIO2_ROS2，先临时移走再执行脚本：
> ```bash
> mv /workspaces/ROS2_FOR_SCOUT_MINI/third_party/fast_lio2_ws/src/FASTLIO2_ROS2 /tmp/
> cd /workspaces/ROS2_FOR_SCOUT_MINI/third_party/fast_lio2_ws/src/livox_ros_driver2
> ./build.sh humble
> mv /tmp/FASTLIO2_ROS2 /workspaces/ROS2_FOR_SCOUT_MINI/third_party/fast_lio2_ws/src/
> ```
>
> 后续如果只修改了 livox 驱动代码，不需要重新执行脚本，直接：
> ```bash
> cd /workspaces/ROS2_FOR_SCOUT_MINI/third_party/fast_lio2_ws
> colcon build --packages-select livox_ros_driver2 --cmake-args -DROS_EDITION=ROS2
> ```

### Sophus 编译安装
```bash
cd /workspaces/ROS2_FOR_SCOUT_MINI/third_party/fast_lio2_ws/src
git clone https://github.com/strasdat/Sophus.git --depth 1 --branch 1.22.10
cd Sophus && mkdir build && cd build
cmake .. -DSOPHUS_USE_BASIC_LOGGING=ON
make -j4
sudo make install
```
> 新的 `Sophus` 依赖 `fmt` 库。上面通过 `-DSOPHUS_USE_BASIC_LOGGING=ON` 关闭 fmt 依赖；也可以在 `CMakeLists.txt` 中添加 `add_compile_definitions(SOPHUS_USE_BASIC_LOGGING)` 达到同样效果，否则会报错。

### FASTLIO2_ROS2
```bash
cd /workspaces/ROS2_FOR_SCOUT_MINI/third_party/fast_lio2_ws/src
git clone https://github.com/liangheming/FASTLIO2_ROS2.git
cd ..
# FASTLIO2_ROS2下的五个功能包
colcon build --packages-select fastlio2 
colcon build --packages-select interface
colcon build --packages-select hba 
colcon build --packages-select localizer 
colcon build --packages-select pgo
```

## 里程计配置

配置文件：[`third_party/fast_lio2_ws/src/FASTLIO2_ROS2/fastlio2/config/lio.yaml`](third_party/fast_lio2_ws/src/FASTLIO2_ROS2/fastlio2/config/lio.yaml)

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `imu_topic` | IMU 话题 | `/livox/imu` |
| `lidar_topic` | 点云话题 | `/livox/lidar` |
| `lidar_min_range` | 最小探测距离（m） | `0.5` |
| `lidar_max_range` | 最大探测距离（m） | `30.0` |
| `scan_resolution` | 扫描降采样分辨率（m） | `0.15` |
| `map_resolution` | 地图体素分辨率（m） | `0.3` |
| `cube_len` | 地图立方体边长（m） | `300` |
| `det_range` | 检测范围（m） | `60` |
| `na / ng` | 加速度/角速度测量噪声 | `0.01` |
| `nba / nbg` | 加速度/角速度 bias 噪声 | `0.0001` |
| `gravity_align` | 重力对齐初始化 | `true` |
| `esti_il` | 在线估计外参 | `false` |
| `t_il` | LiDAR→IMU 平移（m） | `[-0.011, -0.02329, 0.04412]` |
| `r_il` | LiDAR→IMU 旋转矩阵（行优先） | 单位矩阵 |

## mid360 雷达配置

### 硬件连接

Mid-360 通过以太网接口连接工控机，供电 **9-27V DC**：

- 网线直连工控机
- 电源**红正黑负**
- 上电后绿色呼吸灯 = 正常工作

### 网络配置

雷达出厂 IP 为 `192.168.1.1XX`，`XX` = SN 码后两位。本次使用的 Mid-360 SN 尾号 `81`，IP 为 `192.168.1.181`。

工控机端统一设为 `192.168.1.50`（与雷达 SN 无关）：

```bash
# 临时配置
sudo ip addr add 192.168.1.50/24 dev eth0
sudo ip link set eth0 up

# 验证
ping 192.168.1.181
```

### Livox 驱动配置

配置文件：[`third_party/fast_lio2_ws/src/livox_ros_driver2/config/MID360_config.json`](third_party/fast_lio2_ws/src/livox_ros_driver2/config/MID360_config.json)

需要修改的字段：
- `host_net_info` 下 4 个 IP → `192.168.1.50`
- `lidar_configs[0].ip` → `192.168.1.181`

### 启动雷达

```bash
source /workspaces/ROS2_FOR_SCOUT_MINI/third_party/fast_lio2_ws/install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

验证：`ros2 topic list | grep livox` 应看到 `/livox/lidar` 和 `/livox/imu`。

## 配置参数调优

### 根据场景调整 `lio.yaml`

| 场景 | 调整建议 |
|------|---------|
| 户外 300m 赛道 | `cube_len: 500`、`det_range: 100` |
| 特征较少 | 降低 `map_resolution` 到 `0.2` |
| 速度快（> 5m/s） | 增大 `ng` 到 `0.05`，信任 IMU 更多 |
| 外参已知 | `esti_il: false`，填入准确 `t_il`/`r_il` |

### pgo 回环参数（`pgo/config/pgo.yaml`）

| 参数 | 说明 | 建议值 |
|------|------|--------|
| `key_pose_delta_deg` | 关键帧角度间隔（°） | `10` |
| `key_pose_delta_trans` | 关键帧平移间隔（m） | `0.5` |
| `loop_search_radius` | 回环搜索半径（m） | `1.0` |
| `loop_time_tresh` | 回环最小时间间隔（s） | `60` |

### localizer 重定位参数（`localizer/config/localizer.yaml`）

| 参数 | 说明 | 建议值 |
|------|------|--------|
| `rough_scan_resolution` | 粗配准降采样（m） | `0.25` |
| `refine_scan_resolution` | 精配准降采样（m） | `0.1` |
| `rough_score_thresh` | 粗配准得分阈值 | `0.2`（降低放宽匹配）|


## 三维点云投影为 Nav2 二维地图

导航同时保留两类地图：Fast-LIO2 生成的 `.pcd` 三维点云地图用于 ICP 重定位；二维栅格地图用于 Nav2 的全局路径规划。`my_party/map_transformation_ws` 中的 `pointcloud_map_projection` 是本项目自主实现的**离线**地图转换包，参考 [pcd2pgm](https://github.com/LihanChen2004/pcd2pgm) 的处理流程，不依赖或克隆外部仓库源码。

投影链路如下：

```text
GlobalMap.pcd（Fast-LIO2 建图完成后保存）
  → pointcloud_map_projection
      ├─ 按 Z 高度过滤：保留会影响底盘通行的点
      ├─ 半径离群点滤波：去除稀疏噪点
      └─ 自动计算点云边界并栅格化
  → /map（nav_msgs/OccupancyGrid）
  → map_saver_cli
  → competition_map.pgm + competition_map.yaml
```

核心代码位于 [`my_party/map_transformation_ws/src/pointcloud_map_projection`](my_party/map_transformation_ws/src/pointcloud_map_projection)。主要参数如下：

| 参数 | 作用 | 初始值 |
|---|---|---|
| `pcd_file` | 已清理的完整 PCD 地图绝对路径 | 必填 |
| `resolution` | 栅格分辨率 | `0.05 m` |
| `z_min` / `z_max` | 保留为障碍的高度范围 | `0.15–1.20 m` |
| `voxel_leaf_size` | 过滤前体素降采样尺寸 | `0.05 m` |
| `enable_radius_filter` | 是否启用半径离群点滤波 | `true` |
| `radius_search` / `min_neighbors` | 去噪邻域参数 | `0.15 m / 3` |
| `map_padding` | 点云边界外额外保留范围 | `0.50 m` |
| `unobserved_value` | 未观测栅格的占据值 | `-1`（未知） |

首次编译：

```bash
source /opt/ros/humble/setup.bash
cd /workspaces/ROS2_FOR_SCOUT_MINI/my_party/map_transformation_ws
colcon build --packages-select pointcloud_map_projection
```

将 `projection.yaml` 的 `pcd_file` 改为实际地图文件的绝对路径后，运行转换节点：

```bash
source /workspaces/ROS2_FOR_SCOUT_MINI/my_party/map_transformation_ws/install/setup.bash
ros2 launch pointcloud_map_projection projection.launch.py
```

配置文件为 [`projection.yaml`](my_party/map_transformation_ws/src/pointcloud_map_projection/config/projection.yaml)。高度阈值基于重力对齐后的 PCD 坐标系；应先用 RViz 检查滤波结果，再保存最终地图。运行时，ICP 重定位节点动态发布 `map → odom`。

> 转换器无法仅从障碍物点判断未观测区域是否可通行，因此默认将其标记为 unknown（`-1`）。保存 PGM 后，应在地图编辑器中将确认的赛道区域设为 free，并保留赛道外、遮挡区和边界为 unknown/obstacle；不要直接把所有未知格改为 free。

安装 `ros-humble-nav2-map-server` 后，生成 `/map` 即可保存为 Nav2 静态地图：

```bash
ros2 run nav2_map_server map_saver_cli \
  -t /map \
  -f competition_map \
  --fmt png
```

运行时：`competition_map.yaml` 由 Nav2 的 `map_server` 加载，作为全局 costmap；实时点云只用于 local costmap 的障碍物更新。


## 参考链接

- [FASTLIO2_ROS2](https://github.com/liangheming/FASTLIO2_ROS2)
- [FAST-LIO2 原版](https://github.com/hku-mars/FAST_LIO)
- [mid-360 配置教程](https://blog.csdn.net/m0_55117804/article/details/142644882)

## 完整使用流程

下面的流程针对 `third_party/fast_lio2_ws/src/FASTLIO2_ROS2`，使用 Livox MID-360。该仓库包含 5 个 ROS 2 功能包：

| 功能包 | 作用 | 何时使用 |
|--------|------|----------|
| `interface` | 定义保存地图、重定位、地图精化等服务接口 | 被其他包依赖，不需要单独启动 |
| `fastlio2` | 融合 Livox 点云和 IMU，输出实时里程计、轨迹与点云 | 只需要实时里程计，或不需要回环的基础建图时使用 |
| `pgo` | 在 FAST-LIO2 基础上检测回环并进行位姿图优化，同时保存地图 | 正式采集并保存地图时使用 |
| `localizer` | 将当前点云与已有 PCD 地图配准，完成全局重定位 | 已有地图、需要定位运行时使用 |
| `hba` | 使用保存的关键帧和位姿离线精化地图 | 建图结束后，可选使用 |

> `pgo_launch.py` 和 `localizer_launch.py` 都会自动启动 `fastlio2`，不要再同时运行 `lio_launch.py`，否则会出现节点名和话题冲突。

### 1. 每个新终端先加载环境

```bash
cd /workspaces/ROS2_FOR_SCOUT_MINI
source /opt/ros/humble/setup.bash
source third_party/fast_lio2_ws/install/setup.bash
```

如果修改了源码，重新编译并再次加载环境：

```bash
cd /workspaces/ROS2_FOR_SCOUT_MINI/third_party/fast_lio2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --parallel-workers 1 \
  --packages-select interface fastlio2 pgo localizer hba
source install/setup.bash
```

### 2. 启动 Livox MID-360

先确认 `livox_ros_driver2/config/MID360_config.json` 中雷达 IP 和本机 IP 正确，然后启动驱动：

```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

另开终端加载环境后检查数据：

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
```

两个话题都有稳定输出后，才能继续运行 FAST-LIO2。默认输入话题定义在 `fastlio2/config/lio.yaml`。

### 3. 选择一种运行模式

#### 模式 A：仅运行 FAST-LIO2 里程计

适用于测试雷达、查看实时轨迹，或者不需要回环优化的场景：

```bash
ros2 launch fastlio2 lio_launch.py
```

主要输出：

| 输出 | 内容 |
|------|------|
| `/fastlio2/lio_odom` | 激光惯性里程计 |
| `/fastlio2/lio_path` | 运行轨迹 |
| `/fastlio2/body_cloud` | 当前帧去畸变点云，供 PGO 和重定位使用 |
| `/fastlio2/world_cloud` | 世界坐标系中的点云 |

#### 模式 B：回环建图并保存地图（推荐建图方式）

该 launch 会同时启动 `fastlio2`、`pgo` 和 RViz：

```bash
ros2 launch pgo pgo_launch.py
```

驾驶机器人完整覆盖目标区域，并尽量回到经过的位置形成回环。建图结束后另开终端保存：

```bash
mkdir -p /workspaces/ROS2_FOR_SCOUT_MINI/maps/site_01
ros2 service call /pgo/save_maps interface/srv/SaveMaps \
  "{file_path: '/workspaces/ROS2_FOR_SCOUT_MINI/maps/site_01', save_patches: true}"
```

保存成功后目录中应包含：

```text
site_01/
├── map.pcd       # 完整地图，供 localizer 使用
├── poses.txt     # 关键帧位姿，供 HBA 使用
└── patches/      # 关键帧点云，供 HBA 使用
```

如果以后不需要 HBA，可以把 `save_patches` 设为 `false`，只保存 `map.pcd`；需要地图精化时必须设为 `true`。

#### 模式 C：载入已有地图进行重定位

先启动 Livox 驱动，再启动重定位；该 launch 会同时启动 `fastlio2`、`localizer` 和 RViz：

```bash
ros2 launch localizer localizer_launch.py
```

另开终端调用重定位服务。`x/y/z/yaw/pitch/roll` 是机器人在地图中的大致初始位姿，角度单位为弧度：

```bash
ros2 service call /localizer/relocalize interface/srv/Relocalize \
  "{pcd_path: '/workspaces/ROS2_FOR_SCOUT_MINI/maps/site_01/map.pcd', x: 0.0, y: 0.0, z: 0.0, yaw: 0.0, pitch: 0.0, roll: 0.0}"
```

查询重定位结果：

```bash
ros2 service call /localizer/relocalize_check interface/srv/IsValid "{code: 0}"
```

返回 `valid: true` 表示成功。成功后 `localizer` 发布 `map -> lidar` TF；如果失败，应让初始位姿更接近真实位置，或调整 `localizer/config/localizer.yaml` 的配准阈值。

### 4. 使用 HBA 精化地图（可选）

HBA 不是实时建图节点。它读取 PGO 保存的 `patches/` 和 `poses.txt`，优化关键帧位姿并发布精化点云：

```bash
ros2 launch hba hba_launch.py
```

另开终端开始精化：

```bash
ros2 service call /hba/refine_map interface/srv/RefineMap \
  "{maps_path: '/workspaces/ROS2_FOR_SCOUT_MINI/maps/site_01'}"
```

精化结果会发布在 `/hba/map_points`，可在 RViz 中检查。需要保存优化后的位姿时调用：

```bash
ros2 service call /hba/save_poses interface/srv/SavePoses \
  "{file_path: '/workspaces/ROS2_FOR_SCOUT_MINI/maps/site_01/refined_poses.txt'}"
```

### 5. 推荐的实际顺序

```text
配置雷达 IP
  -> 启动 livox_ros_driver2
  -> 检查 /livox/lidar 和 /livox/imu
  -> 启动 pgo_launch.py 完成建图
  -> 调用 /pgo/save_maps 保存 map.pcd、patches 和 poses.txt
  -> （可选）用 hba 精化地图
  -> 下次运行启动 localizer_launch.py
  -> 调用 /localizer/relocalize 加载 map.pcd
  -> valid: true 后使用 map -> lidar TF 进行全局定位
```
