# Jetson 主机操作指南

本指南用于直接在 Jetson 主机运行本项目，不使用 Docker 容器。仓库路径固定为：

```bash
~/auto/ROS2_FOR_SCOUT_MINI
```

项目硬件配置如下：

| 项目 | 配置 |
|---|---|
| 雷达网卡 | `eno1` |
| Jetson 雷达网卡 IP | `192.168.1.50/24` |
| Livox MID-360S IP | `192.168.1.181` |
| Scout Mini USB-CAN | `can2`，`500000 bit/s` |

## 1. 配置并验证 MID-360S 网卡

首次配置或 `eno1` 的连接配置丢失时，在 Jetson 主机执行：

```bash
sudo nmcli connection add type ethernet ifname eno1 con-name mid360s-static ipv4.method manual ipv4.addresses 192.168.1.50/24 ipv4.never-default yes ipv6.method disabled connection.autoconnect yes connection.autoconnect-retries 0
sudo nmcli connection modify mid360s-static connection.autoconnect-priority 100
sudo nmcli connection up mid360s-static
```

已经创建过配置时，只需重新启用：

```bash
sudo nmcli connection up mid360s-static
```

验证网卡和雷达通信：

```bash
ip -4 addr show eno1
ping -c 3 192.168.1.181
```

雷达驱动配置文件为 `third_party/fast_lio2_ws/src/livox_ros_driver2/config/MID360s_config.json`，其中 `host_ip` 必须是 `192.168.1.50`，雷达 `ip` 必须是 `192.168.1.181`。修改后重建 FAST-LIO2 工作空间。

## 2. 配置并验证 Scout Mini USB-CAN

先确认 USB-CAN 驱动已加载、设备已经枚举为 `can2`：

```bash
ip link show can2
```

设置 500K 波特率并启用接口：

```bash
sudo ip link set can2 down
sudo ip link set can2 up type can bitrate 500000
```

检查底盘 CAN 数据：

```bash
ip -details -statistics link show can2
candump -tz can2
```

`candump` 持续收到 CAN 帧后再启动底盘或导航。若系统中不存在 `can2`，应先检查 USB-CAN 适配器、内核驱动和接线；不要误用 Jetson 板载的 `can0`。

## 3. 加载 ROS 2 环境

推荐一次加载全部工作空间：

```bash
source ~/auto/ROS2_FOR_SCOUT_MINI/setup_local.bash
```

手动加载导航环境：

```bash
source /opt/ros/humble/setup.bash
source ~/auto/ROS2_FOR_SCOUT_MINI/third_party/fast_lio2_ws/install/setup.bash
source ~/auto/ROS2_FOR_SCOUT_MINI/third_party/scout_mini_ws/install/setup.bash
source ~/auto/ROS2_FOR_SCOUT_MINI/my_party/navigation_ws/install/setup.bash
```

手动加载地图转换环境：

```bash
source /opt/ros/humble/setup.bash
source ~/auto/ROS2_FOR_SCOUT_MINI/third_party/fast_lio2_ws/install/setup.bash
source ~/auto/ROS2_FOR_SCOUT_MINI/my_party/map_transformation_ws/install/setup.bash
```

## 4. 重新构建

修改 C++、`CMakeLists.txt` 或雷达 JSON 后，按工作空间分别构建：

```bash
source ~/auto/ROS2_FOR_SCOUT_MINI/setup_local.bash
cd ~/auto/ROS2_FOR_SCOUT_MINI/third_party/fast_lio2_ws
colcon build --symlink-install
```

```bash
source ~/auto/ROS2_FOR_SCOUT_MINI/setup_local.bash
cd ~/auto/ROS2_FOR_SCOUT_MINI/third_party/scout_mini_ws
colcon build --symlink-install
```

```bash
source ~/auto/ROS2_FOR_SCOUT_MINI/setup_local.bash
cd ~/auto/ROS2_FOR_SCOUT_MINI/my_party/map_transformation_ws
colcon build --symlink-install
```

```bash
source ~/auto/ROS2_FOR_SCOUT_MINI/setup_local.bash
cd ~/auto/ROS2_FOR_SCOUT_MINI/my_party/navigation_ws
colcon build --symlink-install
```

## 5. 单独检查雷达和底盘

启动 MID-360S 驱动：

```bash
source ~/auto/ROS2_FOR_SCOUT_MINI/setup_local.bash
ros2 launch livox_ros_driver2 msg_MID360s_launch.py
```

另开终端检查数据：

```bash
source ~/auto/ROS2_FOR_SCOUT_MINI/setup_local.bash
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
```

单独启动 Scout Mini 底盘：

```bash
source ~/auto/ROS2_FOR_SCOUT_MINI/setup_local.bash
ros2 launch scout_base scout_mini_base.launch.py port_name:=can2
```

## 6. 启动导航

以室内地图为例：

```bash
source ~/auto/ROS2_FOR_SCOUT_MINI/setup_local.bash
ros2 launch scout_navigation_bringup navigation_system.launch.py map_dir:=~/auto/ROS2_FOR_SCOUT_MINI/maps/outdoor_01 can_port:=can2
```

可替换的地图目录：

```bash
~/auto/ROS2_FOR_SCOUT_MINI/maps/indoor_01
~/auto/ROS2_FOR_SCOUT_MINI/maps/site_01
~/auto/ROS2_FOR_SCOUT_MINI/maps/outdoor_01
```

启动后依次确认雷达点云、FAST-LIO2 里程计、`/scout_status` 与重定位状态正常，再发送导航目标。

## 7. 启动建图

```bash
source ~/auto/ROS2_FOR_SCOUT_MINI/setup_local.bash
ros2 launch scout_navigation_bringup mapping.launch.py start_base:=true can_port:=can2
```
