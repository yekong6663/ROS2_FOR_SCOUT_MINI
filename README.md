# 参考构建
官方仓库[链接](https://github.com/agilexrobotics/scout_ros2)
# 补充安装
## ASIO库
一个跨平台的 C++ 网络库，用于实现高效的异步通信。它可以处理网络、串口等 I/O 操作，而不会阻塞程序的其他任务。
```bash
sudo apt update
sudo apt install -y libasio-dev
```
docker容器中下载慢一定要配置代理，以及根据代理进行配置
```bash
sudo bash -c 'cat > /etc/apt/apt.conf.d/99proxy <<EOF
Acquire::http::Proxy "http://127.0.0.1:7890";
Acquire::https::Proxy "http://127.0.0.1:7890";
EOF'
```

# CAN总线配置通讯

> **关键限制：`modprobe` 加载内核模块必须在 WSL2 中执行，容器无法绕过。**
> 其余步骤（配置 CAN 接口、测试通信、ROS2 驱动）均可在容器内完成，无需切换环境。
>
> | 步骤 | 执行位置 | 原因 |
> |------|---------|------|
> | 内核编译 + `modprobe gs_usb` | **WSL2（仅一次）** | 容器共享宿主机内核，无法加载模块 |
> | `ip link set can0` / `candump` | 容器内 | 已配置 `--cap-add=NET_ADMIN` + `--network=host` |
> | ROS2 驱动（scout_base） | 容器内 | 同容器环境 |

## 硬件连接
使用 USB-CAN 适配器（如 CANable/周立功等）将工控机连接至 Scout Mini 底盘的 CAN 接口。底盘 CAN 接口定义参考官方手册。

**Scout Mini 底盘 CAN 默认参数：**
- 波特率：500K bps（部分版本为 250K，以实际手册为准）

## 1. 安装 CAN 工具（容器内执行）
```bash
sudo apt update
sudo apt install -y can-utils
```

## 2. 使能 gs_usb 内核模块（WSL2 中执行，仅此一步需要 WSL2）

**Linux 原生系统（Ubuntu/Debian）：**
```bash
sudo modprobe gs_usb
```

**WSL2 下需要重新编译内核（WSL2 默认不带 gs_usb 驱动）：**

```bash
# 1. 安装编译依赖
sudo apt update
sudo apt install -y build-essential flex bison libssl-dev libelf-dev bc git wget

# 2. 克隆 WSL2 内核源码
git clone https://github.com/microsoft/WSL2-Linux-Kernel.git
cd WSL2-Linux-Kernel

# 3. 在配置中启用 CAN 和 gs_usb
make KCONFIG_CONFIG=Microsoft/config-wsl menuconfig
```
在 menuconfig 界面中依次进入并启用（按 Y 设为 `*` 或 M）：
```
Device Drivers → Network device support → CAN Bus support → CAN Device Drivers
  → CAN USB interfaces
    → Geschwister Schneider UG CAN USB interfaces (gs_usb)
```

也可以直接编辑配置：
```bash
sed -i 's/# CONFIG_CAN is not set/CONFIG_CAN=y/' Microsoft/config-wsl
echo "CONFIG_CAN_GS_USB=m" >> Microsoft/config-wsl
echo "CONFIG_CAN_DEV=y" >> Microsoft/config-wsl
echo "CONFIG_CAN_RAW=y" >> Microsoft/config-wsl
```

```bash
# 4. 修复 WSL2 内核与新版 GCC 的兼容问题并编译

# 修复 hv_balloon.c 中的指针类型转换警告（新版 GCC 将其视为错误）
sed -i 's/return dmabuf;/return (void *)(uintptr_t)dmabuf;/' drivers/hv/hv_balloon.c
sed -i 's/__u64 dmabuf = (__u64)send_page;/__u64 dmabuf = (__u64)(uintptr_t)send_page;/' drivers/hv/hv_balloon.c

make KCONFIG_CONFIG=Microsoft/config-wsl -j$(nproc)

# 5. 将编译好的内核复制到 Windows 用户目录
cp arch/x86/boot/bzImage /mnt/c/Users/<你的用户名>/ws2_kernel

# 6. 在 Windows 用户目录下创建 .wslconfig 文件
# 路径：C:\Users\<你的用户名>\.wslconfig
# 内容：
# [wsl2]
# kernel=C:\\Users\\<你的用户名>\\ws2_kernel

# 7. PowerShell 中重启 WSL
# wsl --shutdown
# 然后重新打开 WSL 终端

# 8. 验证模块是否可用
sudo modprobe gs_usb
```
> 详细步骤参考 [WSL2 内核编译官方文档](https://learn.microsoft.com/en-us/community/content/wsl-user-msft-kernel-v6)

## 3. 设置 500K 波特率并使能 CAN-to-USB 适配器（容器内执行）
```bash
sudo ip link set can0 up type can bitrate 500000
```

## 4. 验证 CAN 设备（容器内执行）
```bash
ifconfig -a
```
正常情况下应能看到 `can0` 设备。

## 5. 测试 CAN 通信（容器内执行）

若此时 CAN-to-USB 已经与 SCOUT MINI 2.0 机器人相连，且小车已开启，使用 `candump` 即可监听来自底盘的数据：

```bash
candump can0
```
> 收到 CAN 帧即表示通信正常，可继续下一步启动 ROS2 驱动。

## 6. 启动底盘 ROS2 驱动（Docker 容器内执行）

```bash
# 编译工作空间
cd /workspaces/ROS2_For_SCOUT_MINI
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
ros2 topic pub --rate 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.0}}"
```

## 常见问题排查
- **`can0` 不存在**：检查 USB-CAN 适配器是否正确插入，`lsusb` 确认设备识别，`dmesg` 查看驱动加载情况
- **状态为 `BUS-OFF`**：波特率不匹配或接线错误，确认底盘波特率后重新配置
- **启动节点报 `Failed to connect`**：确认 CAN 线连接无误，底盘已上电，`candump can0` 能收到数据
- **底盘不动**：检查 `ros2 topic echo /scout_status` 中 `control_mode` 是否为遥控模式，底盘是否上电且急停开关已解除