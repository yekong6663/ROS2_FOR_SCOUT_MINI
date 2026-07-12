# 工作空间结构

```
ROS2_FOR_SCOUT_MINI/
├── deps_ws/                    # 第三方依赖工作空间（只编译一次）
│   ├── src/
│   │   ├── FASTLIO2_ROS2/      # FAST-LIO2 完整版（里程计+回环+重定位+地图优化）
│   │   ├── livox_ros_driver2/  # Livox ROS2 雷达驱动
│   │   ├── scout_ros2/         # Scout Mini 底盘驱动
│   │   ├── ugv_sdk/            # 底盘通信 SDK
│   │   ├── Livox-SDK2/         # Livox 底层 SDK（cmake 安装）
│   │   └── Sophus/             # 李群/李代数库（cmake 安装）
│   ├── build/
│   └── install/
├── src/                        # 你自己的 ROS2 包（频繁修改）
│   └── my_package/
├── build/
├── install/
└── log/
```

> **设计思路**：两个独立工作空间。第三方包在 `deps_ws/`，只编译一次；你自己的包在 `src/`，修改后单独编译。通过 overlay 叠加使用：
> ```bash
> source deps_ws/install/setup.bash   # 先 source 依赖
> source install/setup.bash           # 再 source 自己的
> ```


# SCOUT MINI底盘


## 1. 基本代码
官方仓库[链接](https://github.com/agilexrobotics/scout_ros2)

```bash
# 克隆到底盘依赖工作空间
cd /workspaces/ROS2_FOR_SCOUT_MINI/deps_ws/src
git clone https://github.com/westonrobot/ugv_sdk.git --depth 1
git clone https://github.com/westonrobot/scout_ros2.git --depth 1

# 编译依赖工作空间
cd /workspaces/ROS2_FOR_SCOUT_MINI/deps_ws
# 注意scout_ros2不是功能包名，其之下有多个功能包
colcon build --packages-select scout_msgs scout_description scout_base ugv_sdk
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
sudo apt update && sudo apt install -y iproute2 //安装iproute2
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
# 编译依赖工作空间
cd /workspaces/ROS2_FOR_SCOUT_MINI
colcon build --base-paths deps_ws/src --install-base deps_ws/install --build-base deps_ws/build
source deps_ws/install/setup.bash

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
//开启底盘运动结点
ros2 launch scout_base scout_base.launch.py is_scout_mini:=true
//使用键盘控制运行
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
键盘控制运行表

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

### Sophus 编译安装

```bash
cd /workspaces/ROS2_FOR_SCOUT_MINI/deps_ws/src
git clone https://github.com/strasdat/Sophus.git --depth 1 --branch 1.22.10
cd Sophus && mkdir build && cd build
cmake .. -DSOPHUS_USE_BASIC_LOGGING=ON
make -j4
sudo make install
```
新的`Sophus`依赖fmt，可以在`CMakeLists.txt`中添加`add_compile_definitions(SOPHUS_USE_BASIC_LOGGING)`去除，否则会报错
### Livox SDK2

```bash
cd /workspaces/ROS2_FOR_SCOUT_MINI/deps_ws/src
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2 && mkdir build && cd build
cmake .. && sudo make install -j4
```

### livox_ros_driver2

```bash
cd /workspaces/ROS2_FOR_SCOUT_MINI/deps_ws/src
git clone -b feature/use-standard-unit https://github.com/Ericsii/livox_ros_driver2.git
cd ..
colcon build --symlink-install --packages-select livox_ros_driver2
```

### FASTLIO2_ROS2

```bash
cd /workspaces/ROS2_FOR_SCOUT_MINI/deps_ws/src
git clone https://github.com/liangheming/FASTLIO2_ROS2.git
cd ..
# FASTLIO2_ROS2下的五个功能包
colcon build --packages-select fastlio2 
colcon build --packages-select hba 
colcon build --packages-select interface
colcon build --packages-select localizer 
colcon build --packages-select pgo
```
### 编译自己的包

```bash
source /workspaces/ROS2_FOR_SCOUT_MINI/deps_ws/install/setup.bash
cd /workspaces/ROS2_FOR_SCOUT_MINI
colcon build
source install/setup.bash
```

## 里程计配置

配置文件：[`deps_ws/src/FASTLIO2_ROS2/fastlio2/config/lio.yaml`](deps_ws/src/FASTLIO2_ROS2/fastlio2/config/lio.yaml)

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

配置文件：[`deps_ws/src/livox_ros_driver2/config/MID360_config.json`](deps_ws/src/livox_ros_driver2/config/MID360_config.json)

需要修改的字段：
- `host_net_info` 下 4 个 IP → `192.168.1.50`
- `lidar_configs[0].ip` → `192.168.1.181`

### 启动雷达

```bash
source install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

验证：`ros2 topic list | grep livox` 应看到 `/livox/lidar` 和 `/livox/imu`。

## 完整使用流程

### 1. 仅里程计（最简启动）

```bash
ros2 launch fastlio2 lio_launch.py
```

输出：
- `/fastlio2/lio_odom` — 里程计位姿
- `/fastlio2/body_cloud` — 去畸变点云

### 2. 里程计 + 回环（建图模式，赛前跑一圈用）

```bash
# 终端 1：里程计
ros2 launch fastlio2 lio_launch.py

# 终端 2：回环检测 + 位姿图优化
ros2 launch pgo pgo_launch.py
```

跑完后保存全局地图：

```bash
ros2 service call /pgo/save_maps interface/srv/SaveMaps “{file_path: '/path/to/save_dir', save_patches: true}”
```

生成文件：
- `map.pcd` — 全局点云地图（给 localizer 用）
- `patches/` — 子地图（给 hba 用）
- `poses.txt` — 关键帧位姿

### 3. 里程计 + 重定位（比赛时定位模式）

```bash
# 终端 1：同时启动里程计和重定位节点
ros2 launch localizer localizer_launch.py

# 终端 2：加载地图并执行重定位（xyzyawpitchroll 为起点附近的大致位姿）
ros2 service call /localizer/relocalize interface/srv/Relocalize “{pcd_path: '/path/to/map.pcd', x: 0.0, y: 0.0, z: 0.0, yaw: 0.0, pitch: 0.0, roll: 0.0}”

# 终端 3：检查重定位是否成功
ros2 service call /localizer/relocalize_check interface/srv/IsValid “{code: 0}”
```

重定位成功后，`/tf` 树中发布 `map → lidar` 变换，里程计输出转换为全局坐标系位姿，可直接给导航使用。

### 4. 地图精化（可选，建图后执行）

```bash
ros2 launch hba hba_launch.py
ros2 service call /hba/refine_map interface/srv/RefineMap “{maps_path: '/path/to/save_dir'}”
```

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

## 常见问题

### 编译时内存爆满卡死

```bash
# 限制并行数 + 分步编译依赖工作空间
colcon build --base-paths deps_ws/src --install-base deps_ws/install --build-base deps_ws/build --parallel-workers 1 --packages-select interface
colcon build --base-paths deps_ws/src --install-base deps_ws/install --build-base deps_ws/build --parallel-workers 1 --packages-select fastlio2
colcon build --base-paths deps_ws/src --install-base deps_ws/install --build-base deps_ws/build --parallel-workers 1 --packages-select pgo localizer hba
```

### livox_ros_driver2 编译报错 `Unknown arguments`

`ROS_VERSION` 环境变量未设置。先执行 `source /opt/ros/humble/setup.bash`。

### 雷达无话题输出

1. 检查 `MID360_config.json` IP 是否正确
2. `ping` 雷达 IP 确认网络通
3. 防火墙不拦截端口 56100-56500

### 建图漂移严重

1. 确认 IMU 数据正常（`ros2 topic hz /livox/imu`）
2. 外参是否准确（调整 `t_il`/`r_il`）
3. 开 `pgo` 回环节点

### 重定位失败

1. 确认 `map.pcd` 覆盖了当前位置
2. 降低 `rough_score_thresh` 到 `0.1`
3. 给定的初始猜测（x, y, z, yaw）不要离真实位置太远

## 参考链接

- [FASTLIO2_ROS2](https://github.com/liangheming/FASTLIO2_ROS2)
- [FAST-LIO2 原版](https://github.com/hku-mars/FAST_LIO)
- [mid-360 配置教程](https://blog.csdn.net/m0_55117804/article/details/142644882)
