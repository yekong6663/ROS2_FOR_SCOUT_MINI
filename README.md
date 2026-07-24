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

#### 6. 单独测试底盘驱动（可选）

这里只用于确认底盘通信。整套系统运行时不要按本节启动，统一使用文末的[统一使用指南](#统一使用指南)，以确保 TF 参数正确。

```bash
# 编译底盘工作空间
cd /workspaces/ROS2_FOR_SCOUT_MINI/third_party/scout_mini_ws
colcon build
source install/setup.bash

# 单独测试 Scout Mini 底盘驱动
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

## 3. 底盘单项运动测试

以下命令只用于脱离建图、重定位和导航系统测试底盘；正式运行步骤见文末的[统一使用指南](#统一使用指南)。

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

#### livox_ros_driver
配置文件：[`third_party/fast_lio2_ws/src/livox_ros_driver2/config/MID360s_config.json`](third_party/fast_lio2_ws/src/livox_ros_driver2/config/MID360s_config.json)

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


## 参考链接

- [FASTLIO2_ROS2](https://github.com/liangheming/FASTLIO2_ROS2)
- [FAST-LIO2 原版](https://github.com/hku-mars/FAST_LIO)
- [mid-360 配置教程](https://blog.csdn.net/m0_55117804/article/details/142644882)

## FAST-LIO2 运行模式说明

下面的流程针对 `third_party/fast_lio2_ws/src/FASTLIO2_ROS2`，使用 Livox MID-360。该仓库包含 5 个 ROS 2 功能包：

| 功能包 | 作用 | 何时使用 |
|--------|------|----------|
| `interface` | 定义保存地图、重定位、地图精化等服务接口 | 被其他包依赖，不需要单独启动 |
| `fastlio2` | 融合 Livox 点云和 IMU，输出实时里程计、轨迹与点云 | 只需要实时里程计，或不需要回环的基础建图时使用 |
| `pgo` | 在 FAST-LIO2 基础上检测回环并进行位姿图优化，同时保存地图 | 正式采集并保存地图时使用 |
| `localizer` | 将当前点云与已有 PCD 地图配准，完成全局重定位 | 已有地图、需要定位运行时使用 |
| `hba` | 使用保存的关键帧和位姿离线精化地图 | 建图结束后，可选使用 |

> `pgo_launch.py` 和 `localizer_launch.py` 都会自动启动 `fastlio2`，不要再同时运行 `lio_launch.py`，否则会出现节点名和话题冲突。正式启动命令统一放在文末的[统一使用指南](#统一使用指南)。

FAST-LIO2 的主要输出：

| 输出 | 内容 |
|------|------|
| `/fastlio2/lio_odom` | 激光惯性里程计 |
| `/fastlio2/lio_path` | 运行轨迹 |
| `/fastlio2/body_cloud` | 当前帧去畸变点云，供 PGO 和重定位使用 |
| `/fastlio2/world_cloud` | 世界坐标系中的点云 |

PGO 保存成功后，地图目录应包含：

```text
<地图目录>/
├── map.pcd       # 完整地图，供 localizer 使用
├── poses.txt     # 关键帧位姿，供 HBA 使用
└── patches/      # 关键帧点云，供 HBA 使用
```

重定位成功后 `localizer` 发布 `map → lidar`；返回 `valid: true` 才能继续导航。HBA 是可选的离线地图精化工具，不在正常运行时启动。

# 导航

## 当前完成情况与待办

当前已具备二维静态地图、FAST-LIO2 三维重定位、`map → lidar → body → base_link` 的统一 TF 链，以及完整的 Nav2 bringup。导航配置位于 [`scout_navigation_bringup`](my_party/navigation_ws/src/scout_navigation_bringup)：

- `map_server` 加载转换得到的 `nav2_map.yaml`。
- 全局代价地图使用 `map` 坐标系、静态地图层和膨胀层。
- 局部滚动代价地图使用 `lidar` 坐标系，将实时 `/fastlio2/body_cloud` 接入体素障碍层。
- 全局规划暂用 NavFn 的 A* 模式，局部控制使用 Regulated Pure Pursuit。
- `velocity_smoother` 对导航速度进行限幅和平滑，最终向底盘 `/cmd_vel` 输出。

`ScoutAstarPlanner` 目前仍是插件骨架，`createPlan()` 尚未实现，因此不能加入 `planner_plugins`。当前配置使用已经过启动验证的内置规划器；自定义 A* 属于后续替换项，不影响现阶段实车导航联调。

导航参数基准包括：

| 项目 | 当前值 |
|---|---|
| 机器人 footprint | `0.62 m × 0.60 m`，另加 `0.02 m` padding |
| 最大前进速度 | `0.35 m/s` |
| 最大倒车速度 | `0.15 m/s` |
| 最大角速度 | `0.60 rad/s` |
| 局部地图范围 | `8 m × 8 m` |
| 实时障碍范围 | 标记 `5 m`，清除 `6 m` |
| 膨胀半径 | `0.55 m` |

这些是首次低速联调值。正式比赛前必须根据实车外廓、制动距离、点云噪声和赛道宽度调整 [`nav2_params.yaml`](my_party/navigation_ws/src/scout_navigation_bringup/config/nav2_params.yaml)。

## TF

Nav2 使用唯一坐标链：

```text
map → lidar → body → base_link
```

其中 `map → lidar` 由 PGO 或 localizer 发布，`lidar → body` 由 FAST-LIO2 发布。这里的 `lidar` 是 FAST-LIO2 的局部世界坐标系名称，不是额外的雷达安装坐标系；`body` 是 FAST-LIO2 的 IMU 状态坐标系。

MID-360 安装位置由实物测量和 `C1965.STEP` 装配模型共同确认。以四轮几何中心处的 `base_link` 为基准，X 向车头、Y 向左、Z 向上：

```text
base_link → MID-360 点云原点 O
x = +0.2183 m
y =  0.0000 m
z = +0.1190 m
roll = pitch = yaw = 0
```

FAST-LIO2 当前雷达到 IMU 外参为：

```yaml
t_il: [-0.011, -0.02329, 0.04412]
r_il: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
```

换算后使用的静态 TF 为：

```text
body → base_link
x = -0.2293 m
y = -0.02329 m
z = -0.07488 m
roll = pitch = yaw = 0
```

静态变换由 [`robot_tf.launch.py`](my_party/navigation_ws/src/scout_navigation_plugins/launch/robot_tf.launch.py) 发布。整套系统中必须关闭底盘驱动的 `odom → base_link` TF，但仍保留 `/odom` 话题，避免 `base_link` 同时存在两个父节点。启动与检查命令见文末的[统一使用指南](#统一使用指南)。

## PCD 转 Nav2 二维栅格地图

`pointcloud_map_projection` 是离线转换工具。FAST-LIO2 的 `map.pcd` 继续用于三维重定位；转换得到的 `nav2_map.png` 和 `nav2_map.yaml` 用于 Nav2 全局规划。比赛运行时不需要反复转换静态地图，实时障碍由局部代价地图处理。

功能包路径：[`my_party/map_transformation_ws/src/pointcloud_map_projection`](my_party/map_transformation_ws/src/pointcloud_map_projection)。

### 1. 准备输入

先使用 `/pgo/save_maps` 保存地图，并确保同一地图目录至少包含：

```text
map.pcd
poses.txt
patches/       # 启用车体自反射过滤时必须存在
```

`map.pcd`、`poses.txt` 和 `patches/` 必须来自同一次保存，不能混用。转换流程为：

```text
PCD 或关键帧 patches
  → 可选：在车体坐标系删除自身反射
  → 体素降采样
  → Z 高度过滤
  → 半径离群点过滤
  → poses.txt 轨迹走廊标记为空闲
  → 障碍投影
  → 可选：清理机器人实际驶过的中心足迹
  → 发布 /map
  → map_saver_cli 保存 PNG 和 YAML
```

### 2. 配置

每张地图单独保存一份 YAML 配置。可以复制 [`projection.yaml`](my_party/map_transformation_ws/src/pointcloud_map_projection/config/projection.yaml)，然后修改其中的绝对路径和参数。

必须设置：

| 参数 | 说明 |
|---|---|
| `pcd_file` | PGO 保存的 `map.pcd` 绝对路径 |
| `poses_file` | 与 PCD 配套的 `poses.txt` 绝对路径 |
| `patches_dir` | 关键帧目录；仅在 `remove_self_points: true` 时使用 |
| `output_topic` | 输出二维地图话题，通常为 `/map` |
| `frame_id` | 地图坐标系，通常为 `map` |

地图与滤波参数：

| 参数 | 当前基准 | 调节方法 |
|---|---:|---|
| `resolution` | `0.05 m` | 越小越精细但占用更多内存；大场地可用 `0.05–0.10 m` |
| `z_min` | `0.15 m` | 地面残留多时增大；低矮锥桶缺失时减小 |
| `z_max` | `1.20 m` | 树冠、顶棚进入地图时减小；需要保留高障碍时增大 |
| `voxel_leaf_size` | `0.05 m` | 点云太密或转换太慢时增大，通常不小于 `resolution` |
| `enable_radius_filter` | `true` | 是否删除孤立噪点 |
| `radius_search` | `0.15 m` | 稀疏噪点多时适当增大；细杆消失时减小 |
| `min_neighbors` | `3` | 增大可加强去噪，但可能删除锥桶、细杆等稀疏障碍 |
| `map_padding` | `0.50 m` | 地图边缘额外留白 |
| `unobserved_value` | `-1` | 未观测区域保持未知；不要直接改为自由空间 `0` |

自由空间参数：

| 参数 | 当前基准 | 调节方法 |
|---|---:|---|
| `free_space_radius` | `0.80 m` | 从轨迹中心向两侧生成自由走廊，应小于确认可通行区域的半宽 |
| `trajectory_clear_radius` | `0.0 m` | `0` 表示关闭；存在沿轨迹重复的伪障碍时，可从 `0.25–0.35 m` 开始调节 |

`trajectory_clear_radius` 会在障碍投影后强制清空轨迹中心，必须小于等于 `free_space_radius`，并且不得超过机器人实际驶过的安全足迹，否则可能把真实墙体清掉。

车体自反射过滤参数：

| 参数 | 说明 |
|---|---|
| `remove_self_points` | 为 `true` 时从 `patches/` 重建地图并过滤车体点 |
| `self_filter.min_x/max_x` | 车体在 `body` 坐标系中的前后范围 |
| `self_filter.min_y/max_y` | 车体左右范围 |
| `self_filter.min_z/max_z` | 车体上下范围 |

包围盒只应覆盖小车实体和雷达可见的安装结构。范围过大会删除靠近车辆的真实障碍。

### 3. 转换结果检查

转换和保存命令统一放在文末的[统一使用指南](#统一使用指南)。只修改 YAML 参数不需要重新编译。

终端出现以下统计表示转换完成：

```text
Generated ... map from ... points and ... poses: ... free, ... occupied cells
```

在 RViz 添加 `Map`，话题选择 `/map`：

- 白色：自由空间，Nav2 可以规划通过。
- 黑色：障碍物。
- 灰色：未知区域，默认不可通行。

重点检查白色走廊是否连续、墙体是否完整、轨迹中心是否存在周期性黑点。发现问题时只修改 YAML 参数并重新启动转换节点。

成功保存后生成：

```text
nav2_map.png
nav2_map.yaml
```

检查 YAML 中的 `image`、`resolution` 和 `origin`。Nav2 运行时由 `map_server` 加载 `nav2_map.yaml`；`map.pcd` 仍由 FAST-LIO2 localizer 使用，两者必须属于同一套地图坐标系。

# 统一使用指南

前面的章节用于说明安装、配置和参数。本节是整套系统唯一的正式运行顺序。每条长期运行的 launch 命令都应放在独立终端中；除非步骤明确要求停止，否则该终端要一直保持运行。

`ros2 topic hz` 和 `tf2_echo` 都会持续输出。观察数秒确认正常后按 `Ctrl+C` 只结束当前检查命令，再继续执行下一项。

## 0. 先选择运行模式

整套系统只有三种工作模式，不要把它们混在一起启动：

```text
建图模式：
Livox → FAST-LIO2 → PGO → map.pcd / poses.txt / patches

离线地图转换：
map.pcd / poses.txt / patches → /map → nav2_map.png / nav2_map.yaml

导航模式：
Livox → FAST-LIO2 → localizer → map→lidar TF
                                  ↓
nav2_map.yaml → Nav2 → /cmd_vel → Scout Mini
                     ↑
       /fastlio2/body_cloud 实时避障
```

各节点在不同模式中的要求：

| 节点 | 建图模式 | 离线转换 | 导航模式 |
|---|---|---|---|
| Livox 驱动 | 必需 | 不启动 | 必需 |
| Scout 底盘驱动 | 使用 ROS 控车时必需；原厂遥控器控车时可不启动 | 不启动 | 必需 |
| `body → base_link` 静态 TF | 建议启动，便于统一显示和提前检查 TF | 不启动 | 必需 |
| PGO（已包含 FAST-LIO2） | 必需 | 不启动 | 禁止启动 |
| localizer（已包含 FAST-LIO2） | 禁止启动 | 不启动 | 必需 |
| `lio_launch.py` | 禁止与 PGO/localizer 同时启动 | 不启动 | 禁止与 localizer 同时启动 |
| 地图投影节点 | 不启动 | 必需 | 不启动 |
| Nav2 | 不启动 | 不启动 | 重定位 `valid: true` 后启动 |

> FAST-LIO2/PGO 建图本身只依赖 `/livox/lidar` 和 `/livox/imu`，不使用 Scout 底盘 `/odom`。建图时启动底盘节点只是为了通过 ROS 控制小车移动；若使用原厂遥控器，可以跳过底盘节点。

## 1. 首次构建或修改源码后构建

```bash
# Scout Mini 底盘驱动
cd /workspaces/ROS2_FOR_SCOUT_MINI/third_party/scout_mini_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select scout_base

# Livox 驱动、FAST-LIO2、PGO 和重定位
cd /workspaces/ROS2_FOR_SCOUT_MINI/third_party/fast_lio2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --parallel-workers 1 \
  --packages-select livox_ros_driver2 interface fastlio2 pgo localizer hba

# PCD 转二维栅格地图
cd /workspaces/ROS2_FOR_SCOUT_MINI/my_party/map_transformation_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select pointcloud_map_projection

# TF、Nav2 bringup、导航插件和 Humble lifecycle 兼容 overlay
cd /workspaces/ROS2_FOR_SCOUT_MINI/my_party/navigation_ws
source /opt/ros/humble/setup.bash
source /workspaces/ROS2_FOR_SCOUT_MINI/third_party/fast_lio2_ws/install/setup.bash
source /workspaces/ROS2_FOR_SCOUT_MINI/third_party/scout_mini_ws/install/setup.bash
source /workspaces/ROS2_FOR_SCOUT_MINI/my_party/map_transformation_ws/install/setup.bash
colcon build --symlink-install \
  --packages-select nav2_lifecycle_manager scout_navigation_plugins \
  scout_navigation_bringup \
  --cmake-args -DBUILD_TESTING=OFF
```

## 一键启动（推荐）

一键 launch 位于 `scout_navigation_bringup`。每次打开新终端，先按以下顺序加载所有独立工作空间：

```bash
source /opt/ros/humble/setup.bash
source /workspaces/ROS2_FOR_SCOUT_MINI/third_party/fast_lio2_ws/install/setup.bash
source /workspaces/ROS2_FOR_SCOUT_MINI/third_party/scout_mini_ws/install/setup.bash
source /workspaces/ROS2_FOR_SCOUT_MINI/my_party/map_transformation_ws/install/setup.bash
source /workspaces/ROS2_FOR_SCOUT_MINI/my_party/navigation_ws/install/setup.bash
```

### A. 一键启动建图节点

使用原厂遥控器移动小车：

```bash
ros2 launch scout_navigation_bringup mapping.launch.py
```

该命令默认一次启动：

```text
Livox MID-360s 驱动
FAST-LIO2
PGO
body → base_link 静态 TF
RViz
```

默认不启动 Scout 底盘驱动。需要使用 `/cmd_vel` 或键盘控制时：

```bash
ros2 launch scout_navigation_bringup mapping.launch.py \
  start_base:=true \
  can_port:=can0
```

`mapping.launch.py` 只负责启动建图节点，不会自动决定何时保存地图。采集完成后仍需调用 `/pgo/save_maps`，防止误操作覆盖已有地图。

常用可选参数：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `start_livox` | `true` | 回放 rosbag 时可设为 `false` |
| `start_base` | `false` | ROS 控车时设为 `true` |
| `start_robot_tf` | `true` | 发布 `body → base_link` |
| `start_rviz` | `true` | 无图形界面时设为 `false` |
| `can_port` | `can0` | Scout SocketCAN 接口 |

### B. 输入地图文件夹，一键生成二维地图

地图文件夹至少需要：

```text
<map_dir>/
├── map.pcd
└── poses.txt
```

执行：

```bash
ros2 launch scout_navigation_bringup map_conversion.launch.py \
  map_dir:=/workspaces/ROS2_FOR_SCOUT_MINI/maps/site_01
```

launch 会自动：

1. 读取 `<map_dir>/map.pcd` 和 `<map_dir>/poses.txt`。
2. 发布投影后的 `/map`。
3. 等待地图生成完成。
4. 在同一目录保存 `nav2_map.png` 和 `nav2_map.yaml`。
5. 保存成功后自动退出。

若文件夹中有同一次 PGO 保存产生的非空 `patches/`，可以启用车体自反射过滤：

```bash
ros2 launch scout_navigation_bringup map_conversion.launch.py \
  map_dir:=/workspaces/ROS2_FOR_SCOUT_MINI/maps/site_01 \
  remove_self_points:=true
```

转换参数集中在 [`map_projection_params.yaml`](my_party/navigation_ws/src/scout_navigation_bringup/config/map_projection_params.yaml)。

### C. 准备自动初始化位姿

导航 launch 会自动读取 `<map_dir>/initial_pose.yaml`。可以复制模板：

```bash
cp /workspaces/ROS2_FOR_SCOUT_MINI/my_party/navigation_ws/src/scout_navigation_bringup/config/initial_pose_example.yaml \
  /workspaces/ROS2_FOR_SCOUT_MINI/maps/site_01/initial_pose.yaml
```

然后填写机器人每次启动时在 `map` 中的大致位姿，角度单位为弧度：

```yaml
initial_pose:
  x: 0.0
  y: 0.0
  z: 0.0
  yaw: 0.0
  pitch: 0.0
  roll: 0.0
```

如果文件不存在，launch 会发出警告并使用命令行位姿参数，默认全零。例如：

```bash
ros2 launch scout_navigation_bringup navigation_system.launch.py \
  map_dir:=/workspaces/ROS2_FOR_SCOUT_MINI/maps/site_01 \
  initial_x:=1.0 \
  initial_y:=2.0 \
  initial_yaw:=1.57
```

### D. 只输入地图文件夹，一键启动导航

准备好的完整地图目录为：

```text
<map_dir>/
├── map.pcd             # localizer 三维重定位
├── poses.txt           # 二维地图转换输入
├── patches/            # 可选，自反射过滤输入
├── nav2_map.png        # Nav2 二维地图
├── nav2_map.yaml       # Nav2 map_server 输入
└── initial_pose.yaml   # 推荐，自动初始化位姿
```

启动命令只需指定文件夹：

```bash
ros2 launch scout_navigation_bringup navigation_system.launch.py \
  map_dir:=/workspaces/ROS2_FOR_SCOUT_MINI/maps/site_01
```

launch 会按顺序完成：

```text
启动 Livox + Scout 底盘（publish_odom_tf=false）+ 静态 TF
                            ↓
启动 FAST-LIO2 + localizer
                            ↓
读取 map.pcd 和 initial_pose.yaml，自动调用重定位服务
                            ↓
轮询 /localizer/relocalize_check
                            ↓
valid: true ──→ 读取 nav2_map.yaml 并启动 Nav2 + RViz
valid: false/超时 ──→ 禁止启动 Nav2 并关闭本次 launch
```

常用参数：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `can_port` | `can0` | Scout SocketCAN 接口 |
| `start_livox` | `true` | 外部已启动雷达时设为 `false` |
| `start_base` | `true` | 外部已启动底盘时设为 `false` |
| `start_localization` | `true` | 外部已有 FAST-LIO2/localizer 时设为 `false` |
| `start_rviz` | `true` | 无图形界面时设为 `false` |
| `start_nav2` | `true` | 只测试自动重定位时可设为 `false` |
| `relocalization_timeout` | `120.0` | 等待有效重定位的最长秒数 |

以下章节保留分终端启动方式，用于理解数据流和定位故障。

## 2. 建图（手动分终端）

### 2.1 启动建图节点

终端 1（必需，持续运行）——启动 MID-360s：

```bash
source /opt/ros/humble/setup.bash
source /workspaces/ROS2_FOR_SCOUT_MINI/third_party/fast_lio2_ws/install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360s_launch.py
```

继续前先检查：

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
```

两个话题都必须稳定更新。

终端 2（按移动方式选择）——如果通过 ROS 控制小车，启动底盘并关闭其 TF 发布：

```bash
source /opt/ros/humble/setup.bash
source /workspaces/ROS2_FOR_SCOUT_MINI/third_party/scout_mini_ws/install/setup.bash
ros2 launch scout_base scout_mini_base.launch.py \
  port_name:=can0 \
  publish_odom_tf:=false
```

`publish_odom_tf:=false` 不能省略，否则底盘的 `odom → base_link` 会与 FAST-LIO2 TF 链冲突。使用原厂遥控器移动小车时，建图阶段可以完全跳过终端 2。

终端 3（建议，持续运行）——发布 `body → base_link`：

```bash
source /opt/ros/humble/setup.bash
source /workspaces/ROS2_FOR_SCOUT_MINI/my_party/navigation_ws/install/setup.bash
ros2 launch scout_navigation_plugins robot_tf.launch.py
```

终端 4（必需，持续运行）——启动回环建图：

```bash
source /opt/ros/humble/setup.bash
source /workspaces/ROS2_FOR_SCOUT_MINI/third_party/fast_lio2_ws/install/setup.bash
ros2 launch pgo pgo_launch.py
```

`pgo_launch.py` 已包含 FAST-LIO2，禁止再启动 `lio_launch.py` 或 localizer。

继续前检查：

```bash
ros2 topic hz /fastlio2/lio_odom
ros2 topic hz /fastlio2/body_cloud
```

两个话题都稳定更新后才能移动小车。

### 2.2 移动小车采集地图

- 使用原厂遥控器：直接低速、平稳地移动小车。
- 使用 ROS 键盘控制：保持终端 2 运行，另开终端执行：

```bash
source /opt/ros/humble/setup.bash
source /workspaces/ROS2_FOR_SCOUT_MINI/third_party/scout_mini_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

采集时应覆盖所有计划通行区域，并尽量回到已经经过的位置形成闭环。避免急加速、剧烈旋转、碰撞和长时间停留在缺少几何特征的位置。

## 3. 保存三维地图

建图完成后至少保持终端 1 和终端 4 运行，另开终端执行：

```bash
source /opt/ros/humble/setup.bash
source /workspaces/ROS2_FOR_SCOUT_MINI/third_party/fast_lio2_ws/install/setup.bash
export MAP_NAME=site_01  # 每次建图修改为新的地图名
mkdir -p /workspaces/ROS2_FOR_SCOUT_MINI/maps/${MAP_NAME}
ros2 service call /pgo/save_maps interface/srv/SaveMaps \
  "{file_path: '/workspaces/ROS2_FOR_SCOUT_MINI/maps/${MAP_NAME}', save_patches: true}"
```

只有服务调用成功，并确认目录中存在同一次保存产生的 `map.pcd`、`poses.txt` 和非空 `patches/` 后，才能停止建图节点：

```bash
ls -lh /workspaces/ROS2_FOR_SCOUT_MINI/maps/${MAP_NAME}/map.pcd
ls -lh /workspaces/ROS2_FOR_SCOUT_MINI/maps/${MAP_NAME}/poses.txt
find /workspaces/ROS2_FOR_SCOUT_MINI/maps/${MAP_NAME}/patches -type f | head
```

## 4. 转换并保存二维地图

本步骤完全离线执行，Livox、底盘、PGO、localizer 和 Nav2 都不需要运行。先复制并修改地图专用配置，确保 `pcd_file`、`poses_file` 和 `patches_dir` 指向同一个地图目录，然后启动转换：

```bash
source /opt/ros/humble/setup.bash
source /workspaces/ROS2_FOR_SCOUT_MINI/my_party/map_transformation_ws/install/setup.bash
export PROJECTION_CONFIG=/workspaces/ROS2_FOR_SCOUT_MINI/my_party/map_transformation_ws/src/pointcloud_map_projection/config/site_01_projection.yaml
ros2 launch pointcloud_map_projection projection.launch.py \
  params_file:=${PROJECTION_CONFIG}
```

在 RViz 确认白色自由空间、黑色障碍和灰色未知区域正确。保持转换节点运行，另开终端保存：

```bash
source /opt/ros/humble/setup.bash
export MAP_NAME=site_01  # 与三维地图目录名一致
ros2 run nav2_map_server map_saver_cli \
  -t /map \
  -f /workspaces/ROS2_FOR_SCOUT_MINI/maps/${MAP_NAME}/nav2_map \
  --fmt png
```

只有生成以下两个文件并在 RViz 中确认地图正确后，才能进入导航模式：

```text
maps/site_01/nav2_map.png
maps/site_01/nav2_map.yaml
```

## 5. 重定位

导航模式需要四个长期运行的终端：

| 终端 | 节点 | 要求 |
|---|---|---|
| 1 | Livox 驱动 | 与建图终端 1 相同 |
| 2 | Scout 底盘驱动 | 必须启动，并设置 `publish_odom_tf:=false` |
| 3 | `body → base_link` 静态 TF | 必须启动 |
| 4 | localizer | 必须启动；不要同时启动 PGO 或 `lio_launch.py` |

先按建图章节中的命令启动终端 1、2、3。这里不能因为使用原厂遥控器而跳过底盘节点，因为 Nav2 最终需要通过它向 Scout Mini 执行 `/cmd_vel`。

终端 4（持续运行）——启动 localizer：

```bash
source /opt/ros/humble/setup.bash
source /workspaces/ROS2_FOR_SCOUT_MINI/third_party/fast_lio2_ws/install/setup.bash
ros2 launch localizer localizer_launch.py
```

localizer 已包含 FAST-LIO2，禁止再启动 `pgo_launch.py` 或 `lio_launch.py`。等待 `/fastlio2/lio_odom` 和 `/fastlio2/body_cloud` 稳定输出后，另开一次性命令终端，输入机器人在地图中的大致初始位姿：

```bash
source /opt/ros/humble/setup.bash
source /workspaces/ROS2_FOR_SCOUT_MINI/third_party/fast_lio2_ws/install/setup.bash
export MAP_NAME=site_01  # 与保存地图时一致
ros2 service call /localizer/relocalize interface/srv/Relocalize \
  "{pcd_path: '/workspaces/ROS2_FOR_SCOUT_MINI/maps/${MAP_NAME}/map.pcd', x: 0.0, y: 0.0, z: 0.0, yaw: 0.0, pitch: 0.0, roll: 0.0}"

ros2 service call /localizer/relocalize_check interface/srv/IsValid "{code: 0}"
```

若返回 `valid: false`，检查 PCD 路径、初始位置和 yaw 是否合理，重新调用 `/localizer/relocalize`。只有返回 `valid: true` 后才能进入下一步；终端 1～4 必须继续运行。

## 6. 检查 TF

```bash
source /opt/ros/humble/setup.bash
ros2 run tf2_ros tf2_echo body base_link
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_tools view_frames
```

正确结果为：

```text
map → lidar → body → base_link
```

验收条件：

- `body → base_link` 数值固定。
- `map → base_link` 能持续输出，小车移动时连续变化。
- `base_link` 只能有 `body` 一个父节点。
- TF 树中不应出现底盘发布的 `odom → base_link`。

任一条件不满足都不能启动 Nav2。

## 7. 启动 Nav2

此时以下长期运行节点必须仍然存在：

```text
Livox 驱动
Scout 底盘驱动（publish_odom_tf=false）
body → base_link 静态 TF
localizer（包含 FAST-LIO2）
```

启动前逐项检查：

```bash
ros2 topic hz /fastlio2/body_cloud
ros2 topic hz /fastlio2/lio_odom
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo map base_link
```

全部正常，并且重定位已经返回 `valid: true` 后，终端 5（持续运行）启动 Nav2：

```bash
source /opt/ros/humble/setup.bash
source /workspaces/ROS2_FOR_SCOUT_MINI/my_party/navigation_ws/install/setup.bash
export MAP_NAME=site_01
ros2 launch scout_navigation_bringup navigation.launch.py \
  map:=/workspaces/ROS2_FOR_SCOUT_MINI/maps/${MAP_NAME}/nav2_map.yaml
```

必须 source 导航工作空间，因为其中包含针对当前 Humble `diagnostic_updater` ABI 重新编译的 `nav2_lifecycle_manager` overlay。启动成功时应看到两个 lifecycle manager 都输出：

```text
Managed nodes are active
```

## 8. 导航前检查与发送目标

首次实车测试先架空车轮或把最大速度降到 `0.10 m/s`，确认没有其他节点同时发布 `/cmd_vel`。检查：

```bash
ros2 lifecycle get /map_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 topic hz /fastlio2/body_cloud
ros2 topic info /cmd_vel --verbose
```

前三个节点均应为 `active`。在 RViz 中：

1. Fixed Frame 设为 `map`。
2. 添加 `Map`、`Global Costmap`、`Local Costmap` 和 `PointCloud2`。
3. 确认局部代价地图能标记实时障碍，且机器人 footprint 方向、尺寸正确。
4. 使用 `Nav2 Goal` 发送近距离、无遮挡目标。

先完成直线、原地转向、绕静态障碍和动态障碍停车测试，再逐步提高速度。若局部地图没有实时障碍、TF 跳变、重定位失效或出现多个 `/cmd_vel` 发布者，立即停止导航并排查。

## 9. 停止顺序与异常处理

正常停止时：

1. 先停止 Nav2，确保不再生成运动指令。
2. 停止键盘控制或其他 `/cmd_vel` 发布者。
3. 停止 localizer/PGO。
4. 停止底盘节点。
5. 最后停止 Livox 和静态 TF。

紧急情况下先使用硬件急停或原厂遥控器接管，然后停止 Nav2。不要依赖关闭 RViz 来停车，因为 RViz 不是 `/cmd_vel` 执行节点。

以下任一情况出现时禁止继续发送目标：

- localizer 返回 `valid: false`，或 `map → base_link` 消失/跳变。
- `/fastlio2/body_cloud` 停止更新，局部代价地图无法标记实时障碍。
- `/odom` 停止更新。
- `/cmd_vel` 存在非预期的多个发布者。
- Nav2 lifecycle 节点不是 `active`。
- 地图、机器人位置、方向或 footprint 明显不正确。
