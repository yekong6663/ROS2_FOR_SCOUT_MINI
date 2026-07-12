# 工作空间结构

```
ROS2_FOR_SCOUT_MINI/
├── src/                        # colcon 工作源码目录
│   ├── FAST_LIO -> ../dependencies/FAST_LIO     # 符号链接 → 第三方
│   ├── livox_ros_driver2 -> ../dependencies/livox_ros_driver2
│   ├── scout_ros2 -> ../dependencies/scout_ros2
│   └── my_package/             # 你自己的 ROS2 包放这里
├── dependencies/               # 第三方克隆源码（只读，不直接修改）
│   ├── FAST_LIO/
│   ├── livox_ros_driver2/
│   ├── scout_ros2/
│   ├── Livox-SDK2/
│   └── ugv_sdk/
├── build/                      # colcon 构建产物
├── install/                    # colcon 安装产物
└── log/                        # 构建日志
```
> **设计思路**：第三方代码物理上放在 `dependencies/`，通过符号链接映射到 `src/`，这样 colcon 能找到它们，而你的代码和第三方代码分开管理，互不干扰。


# SCOUT MINI底盘


## 1. 基本代码
官方仓库[链接](https://github.com/agilexrobotics/scout_ros2)
实际上只需要将官方的SDK克隆到dependencies文件夹下

```bash
cd /workspaces/ROS2_FOR_SCOUT_MINI/dependencies
git clone https://github.com/westonrobot/ugv_sdk.git
git clone https://github.com/westonrobot/scout_ros2.git
# 创建符号链接，让 colcon 能找到这些包
ln -s ../dependencies/scout_ros2 /workspaces/ROS2_FOR_SCOUT_MINI/src/scout_ros2
cd /workspaces/ROS2_FOR_SCOUT_MINI
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
# 编译工作空间
cd /workspaces/ROS2_FOR_SCOUT_MINI
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
//开启底盘运动结点
ros2 launch scout_base scout_base.launch.py is_scout_mini:=true
//使用键盘控制运行
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
键盘控制运行表

# 建图SLAM 
构建参考开源库：[FAST_LIO2](https://github.com/hku-mars/FAST_LIO/blob/ROS2/README.md)

## FAST_LIO2的构建
### 1. 先决条件
#### PCL 和 Eigen
```bash
sudo apt update
sudo apt install -y libpcl-dev libeigen3-dev
```
#### livox_ros_driver2

##### 第一步：安装 Livox SDK2（底层驱动）

这是所有 Livox 雷达的基础驱动库，必须先安装。

```bash
cd /workspaces/ROS2_FOR_SCOUT_MINI/dependencies
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2
mkdir build && cd build
cmake .. && sudo make install -j4
```

##### 第二步：安装 `livox_ros_driver2`（ROS 2 驱动）

这是 FAST-LIO2 真正依赖的 ROS 2 驱动包。
使用官方推荐的分支：

```bash
cd /workspaces/ROS2_FOR_SCOUT_MINI/dependencies
git clone -b feature/use-standard-unit https://github.com/Ericsii/livox_ros_driver2.git
# 创建符号链接，让 colcon 能找到这个包
ln -s ../dependencies/livox_ros_driver2 /workspaces/ROS2_FOR_SCOUT_MINI/src/livox_ros_driver2
cd /workspaces/ROS2_FOR_SCOUT_MINI
colcon build --packages-select livox_ros_driver2
```

##### 第三步：配置 source（让每次终端都能找到驱动）

官方文档强调的 **“必须 source”**，就是为了让 FAST-LIO2 能找到 `livox_ros_driver2` 的消息类型。最稳妥的方式是加到 `~/.bashrc`：

```bash
echo "source /workspaces/ROS2_FOR_SCOUT_MINI/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**验证是否生效**：

```bash
ros2 interface list | grep CustomMsg
```

如果能输出 `livox_ros_driver2/msg/CustomMsg`，说明驱动已经正确加载。

##### 第四步：测试雷达数据是否正常

启动雷达驱动，确认 MID-360 能正常发数据：

```bash
# 启动雷达
ros2 launch livox_ros_driver2 msg_MID360_launch.py

# 打开另一个终端，检查话题
ros2 topic list | grep livox
```

如果能看见 `/livox/lidar` 和 `/livox/imu`，说明驱动安装成功。
### 2. 构建 FAST_LIO2
克隆仓库并进行 colcon 构建：

```bash
cd /workspaces/ROS2_FOR_SCOUT_MINI/dependencies
git clone https://github.com/Ericsii/FAST_LIO.git --recursive
# 创建符号链接，让 colcon 能找到这个包
ln -s ../dependencies/FAST_LIO /workspaces/ROS2_FOR_SCOUT_MINI/src/FAST_LIO
cd /workspaces/ROS2_FOR_SCOUT_MINI
rosdep update
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

**注意事项**：
- 请在构建之前先 source 好 `livox_ros_driver2`（按照 1.3 节操作）
- 如果要使用自定义的 PCL 版本，请将以下行添加到 `~/.bashrc`：
  ```bash
  export PCL_ROOT={CUSTOM_PCL_PATH}
  ```
### 3. 运行 FAST-LIO2

#### 注意事项

- **IMU 与 LiDAR 必须时间同步**，这对建图精度至关重要。
- 如果出现 `"Failed to find match for field 'time'"` 警告，说明点云数据中缺少每个点的时间戳，这将影响运动畸变校正。
- 如果已知外参，建议将 `extrinsic_est_en` 设置为 `false`。外参初始化可参考 [Robust Real-time LiDAR-inertial Initialization](https://github.com/hku-mars/LiDAR_IMU_Init)。

#### 3.1 使用 ROS Launch 运行

连接 Livox LiDAR 后，执行以下命令：

```bash
cd /workspaces/ROS2_FOR_SCOUT_MINI
source install/setup.bash
ros2 launch fast_lio mapping.launch.py config_file:=avia.yaml
```

`config_file` 参数可根据需要修改为 `config` 目录下的其他 yaml 文件。

**启动 Livox 雷达驱动（以 MID360 为例）**：

```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

> **注意**：对于 Livox 系列雷达，FAST-LIO 仅支持通过 `livox_lidar_msg.launch` 采集的数据，因为只有其 `livox_ros_driver2/CustomMsg` 数据结构包含每个激光点的时间戳，这对运动畸变校正是必需的。`livox_lidar.launch` 目前无法提供该信息。

如果需要修改雷达帧率，请调整 `livox_lidar_msg.launch` 中的 `publish_freq` 参数，然后重新编译 `livox_ros_driver` 包。

#### 3.2 针对 Livox 系列 + 外部 IMU 的配置

`mapping_avia.launch` 理论上支持 MID-70、MID-40 等 Livox 雷达，但运行前需要配置 `config/mid360.yaml` 中的以下参数：

| 参数 | 说明 |
| :--- | :--- |
| `lid_topic` | LiDAR 点云话题名称 |
| `imu_topic` | IMU 话题名称 |
| `extrinsic_T` | 平移外参（LiDAR 在 IMU 坐标系下的位置） |
| `extrinsic_R` | 旋转外参（仅支持旋转矩阵） |

> **外参定义**：FAST-LIO 中的外参定义为 LiDAR 在 IMU 本体坐标系下的位姿（位置和旋转矩阵）。具体数值可参考雷达官方手册。

> **软件时间同步**：FAST-LIO 为 Livox 雷达提供了简单的软件时间同步功能，可将 `time_sync_en` 参数设为 `true` 启用。但仅在无法实现硬件时间同步时使用，因为软件同步无法保证精度。

#### 3.3 PCD 文件保存

将 launch 文件中的 `pcd_save_enable` 设置为 `1`。FAST-LIO 终止后，所有扫描（全局坐标系下）将累积保存到 `FAST_LIO/PCD/scans.pcd` 文件中。

使用 `pcl_viewer scans.pcd` 可可视化点云。运行时按键盘数字键可切换显示模式：

| 按键 | 显示模式 |
| :--- | :--- |
| `1` | 全部随机颜色 |
| `2` | X 轴值 |
| `3` | Y 轴值 |
| `4` | Z 轴值 |
| `5` | 强度值 |

好的，我已根据你提供的笔记和CSDN教程，对`mid360的使用`部分进行了完善和整合。更新后的内容如下，主要补充了驱动配置、FAST-LIO适配及常见问题排查等关键信息。

---

## mid360的使用

[> mid-360的构建参考](https://blog.csdn.net/m0_55117804/article/details/142644882)

### 硬件连接

Mid-360 雷达通过以太网接口连接工控机，供电电压为 **9-27V DC**。使用前需确认：

- 雷达网口与工控机网口通过网线直连
- 雷达电源线正确连接（**红正黑负**）
- 上电后雷达指示灯正常（**绿色呼吸灯**表示正常工作）

### 网络配置

Mid-360 使用静态 IP 通信，需将工控机网卡配置为与雷达同一网段。

#### 1. 确认雷达 IP

雷达出厂默认 IP 为 **`192.168.1.1XX`**，其中 **`XX` 是雷达 SN 码的最后两位数字**。

> 本次使用的 mid-360 的 SN 码尾号为 `81`，则 IP 为 `192.168.1.181`。也可通过 Livox Viewer 软件查看和修改。

#### 2. 配置工控机网卡 IP

先用 `ip a` 查看连接雷达的网卡名称（如 `eth0`、`enp2s0` 等），然后配置为与雷达同一网段的静态 IP。

**临时配置（测试用）：**

```bash
sudo ip addr add 192.168.1.50/24 dev eth0   # eth0 改为实际网卡名
sudo ip link set eth0 up
```

**永久配置（图形化界面）：**

在 Ubuntu 设置中，按照下图配置 IPv4 地址、子网掩码和网关。**无论雷达序列号是多少，工控机端的 IP 都可以统一设为 `192.168.1.50`**。

![图像](https://i-blog.csdnimg.cn/direct/f28283e337234a26ac3fc1fdcc87b5d3.png)
![图像](https://xju-hurricane-team.github.io/Vision/navigation/lidar/picture/lidar_ubuntu.png)

#### 3. 验证网络连通性

```bash
ping 192.168.1.181   # 替换为你的雷达实际 IP
```

如果能 ping 通，说明网络配置正确。

### Livox 驱动配置

#### 1. 配置文件说明

Mid-360 的驱动配置文件位于 `dependencies/livox_ros_driver2/config/MID360_config.json`，关键参数：

| 参数 | 说明 | 本次配置值 |
| :--- | :--- | :--- |
| `host_net_info.cmd_data_ip` | 工控机接收雷达命令数据的 IP | `192.168.1.50` |
| `host_net_info.push_msg_ip` | 工控机接收雷达推送消息的 IP | `192.168.1.50` |
| `host_net_info.point_data_ip` | 工控机接收点云数据的 IP | `192.168.1.50` |
| `host_net_info.imu_data_ip` | 工控机接收 IMU 数据的 IP | `192.168.1.50` |
| `lidar_configs[0].ip` | 雷达 IP 地址 | `192.168.1.181` |
| `lidar_configs[0].pcl_data_type` | 点云数据类型（1=笛卡尔坐标） | `1` |
| `lidar_configs[0].pattern_mode` | 扫描模式（0=标准模式） | `0` |

将前四个关于主机的 ip （即`host_net_info`下）全部修改为`192.168.1.50`，将关于雷达的配置（即`lidar_configs`下）的 ip 修改为对于雷达的配置，即`192.168.1.181`.


#### 2. 外参配置

在`lidar_configs`下的雷达在机器人上的安装外参在 `lidar_configs[0].extrinsic_parameter` 中配置：

```json
"extrinsic_parameter" : {
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0,
    "x": 0,
    "y": 0,
    "z": 0
}
```

单位为：角度（度）、平移（毫米）。如果需要精确建图，建议通过标定工具获取准确外参后再填入。

#### 3. 启动雷达驱动

```bash
# 确保 source 了工作空间
source install/setup.bash

# 启动 Mid-360 雷达驱动
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

> 必须使用 `msg_MID360_launch.py` 启动驱动，因为只有 `livox_ros_driver2/CustomMsg` 消息格式包含每个激光点的时间戳，这对运动畸变校正至关重要。

#### 4. 验证雷达数据

打开另一个终端，检查话题是否正常发布：

```bash
source install/setup.bash

# 查看 livox 相关话题
ros2 topic list | grep livox
```

正常情况下应看到：
```
/livox/lidar
/livox/imu
```

进一步查看数据频率：
```bash
ros2 topic hz /livox/lidar   # 点云数据，通常 10Hz
ros2 topic hz /livox/imu      # IMU 数据，通常 100Hz
```

### FAST_LIO 中的 Mid-360 配置

#### 1. 配置文件 `mid360.yaml` 关键参数

雷达驱动正常工作后，需要检查 FAST_LIO 的配置文件 `dependencies/FAST_LIO/config/mid360.yaml` 中的参数是否与实际匹配：

| 参数 | 说明 | 本次配置值 |
| :--- | :--- | :--- |
| `common.lid_topic` | LiDAR 点云话题名称 | `/livox/lidar` |
| `common.imu_topic` | IMU 话题名称 | `/livox/imu` |
| `common.time_sync_en` | 软件时间同步（仅无硬件同步时开启） | `false` |
| `common.time_offset_lidar_to_imu` | LiDAR 到 IMU 的时间偏移（秒） | `0.0` |
| `preprocess.lidar_type` | 雷达类型（1=Livox） | `1` |
| `preprocess.scan_line` | 扫描线数 | `4` (Mid-360 为 4 线) |
| `preprocess.blind` | 盲区距离（米） | `0.5` |
| `preprocess.scan_rate` | 扫描频率（Hz） | `10` |
| `mapping.extrinsic_T` | LiDAR 在 IMU 坐标系下的平移（米） | `[-0.011, -0.02329, 0.04412]` |
| `mapping.extrinsic_R` | LiDAR 在 IMU 坐标系下的旋转矩阵 | 单位矩阵 |
| `mapping.extrinsic_est_en` | 是否在线估计外参 | `true` (无精确外参时可开启) |
| `pcd_save.pcd_save_en` | 是否保存 PCD 文件 | `true` |

> **外参说明**：FAST-LIO 中 `extrinsic_T` 和 `extrinsic_R` 定义的是 **LiDAR 在 IMU 本体坐标系下的位姿**（即 LiDAR → IMU 的变换）。如果此参数不准确，建图会出现重影或变形。
