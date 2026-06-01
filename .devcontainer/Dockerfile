# ==================== 基础镜像 ====================
# 使用 OSRF 官方提供的 ROS 2 Humble 桌面完整版镜像
# 该镜像基于 Ubuntu 22.04 (Jammy)，已预装：
#   - ROS 2 Humble 核心
#   - Gazebo 11（用于仿真）
#   - rviz2、rqt、turtlesim 等可视化与调试工具
#   - 各种传感器驱动和常用库
# 大小约 3.5 GB，是开发 ROS 2 应用的推荐起点
FROM osrf/ros:humble-desktop-full

# ==================== 安装额外开发工具 ====================
# 更新软件源并安装 --no-install-recommends 避免安装非必需的建议包，减小镜像体积
RUN apt-get update && apt-get install -y --no-install-recommends \
    # colcon 构建工具（ROS 2 的编译系统，类似 ROS 1 的 catkin）
    python3-colcon-common-extensions \
    # rosdep 依赖管理工具（自动安装功能包的系统依赖）
    python3-rosdep \
    # pip 用于安装 Python 包
    python3-pip \
    # vcstool 用于管理多个 git 仓库（.repos 文件）
    python3-vcstool \
    # ROS 开发工具集（包含 ros2cli、launch 等）
    ros-dev-tools \
    # GNU 调试器，用于 C++ 断点调试
    gdb \
    # ping 命令，测试网络连通性
    iputils-ping \
    # 命令行下载工具
    curl wget \
    # OpenGL 工具集（包含 glxinfo 等），用于诊断图形加速问题
    mesa-utils \
    # CAN 总线调试工具（candump、cansend 等），底盘通信必需
    can-utils \
    # 网络接口管理工具（ifconfig），CAN 设备验证必需
    net-tools \
    # 清理 apt 缓存，减小镜像体积
    && rm -rf /var/lib/apt/lists/*

# ==================== rosdep 初始化（已注释，按需启用） ====================
# 下面两行被注释，因为在国内网络环境下 rosdep update 非常慢且容易失败。
# 如果你需要自动依赖安装，可以取消注释，但建议改用 rosdepc（国内镜像工具）。
# RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi \
#    && rosdep update

# ==================== 创建非 root 用户 ====================
# ROS 2 不推荐在 root 下运行某些节点（例如 rviz 可能会警告）。
# 创建一个名为 ros 的用户，并赋予免密 sudo 权限，方便安装软件。
# 创建 ros 用户并设置权限
# -m 创建家目录，-s 指定默认 shell
# 设置密码（实际登录不会用到）
# 加入 sudo 组
# 创建 render 组（若不存在）
# 加入串口、视频、渲染组，便于访问硬件
#
# 接收宿主机 docker 组 GID（用于 Docker-in-Docker 场景）
# 默认值 999 是常见 Linux 发行版中 docker 组的 GID
ARG DOCKER_GID=999
RUN useradd -m -s /bin/bash ros \
    && echo "ros:ros" | chpasswd \
    && usermod -aG sudo ros \
    && echo "%sudo ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers \
    && groupadd -f render \
    && usermod -aG dialout,video,render ros \
    && groupadd -f -g ${DOCKER_GID} docker \
    && usermod -aG docker ros

# ==================== 自动加载 ROS 2 环境 ====================
# 在 ros 用户的 .bashrc 中 source ROS 2 的环境，这样每次打开终端无需手动执行
RUN echo "source /opt/ros/humble/setup.bash" >> /home/ros/.bashrc

# ==================== 自动加载工作空间环境（如果已编译） ====================
# 接收构建参数 WORKSPACE，该参数由 .devcontainer.json 传递，值为容器内工作区路径
# 例如 /workspaces/ROS2_For_SCOUT_MINI
ARG WORKSPACE
# 如果工作区内已经编译（存在 install/setup.bash），则自动 source
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc

# ==================== 终端美化与快捷键别名 ====================
RUN sed -i 's/#force_color_prompt=yes/force_color_prompt=yes/' /home/ros/.bashrc \
    # 开启彩色命令行提示符
    && echo "alias ls='ls --color=auto'" >> /home/ros/.bashrc \
    && echo "alias ll='ls -alF --color=auto'" >> /home/ros/.bashrc \
    && echo "alias grep='grep --color=auto'" >> /home/ros/.bashrc \
    # 常用编译别名：cb = 完整编译工作区（符号链接方式）
    && echo "alias cb='colcon build --symlink-install'" >> /home/ros/.bashrc \
    # cbs = 只编译指定的包（例如 cbs scout_ros2）
    && echo "alias cbs='colcon build --symlink-install --packages-select'" >> /home/ros/.bashrc \
    # 启用 colcon 的 cd 快捷命令（例如 colcon cd scout_ros2）
    && echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> /home/ros/.bashrc \
    && echo "export _colcon_cd_root=/opt/ros/humble/" >> /home/ros/.bashrc \
    # 启用 colcon 的命令行自动补全
    && echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/ros/.bashrc

# ==================== ROS 2 环境变量 ====================
# 设置 ROS_DOMAIN_ID = 0，这是 ROS 2 的域 ID，用于隔离不同 ROS 2 系统。
# 同一网络下，不同域 ID 的节点不会互相通信。多机器人比赛时可修改。
ENV ROS_DOMAIN_ID=0

# 注意：没有指定 USER，镜像构建后默认仍是 root 用户，
# 但 .devcontainer.json 中会通过 "containerUser": "ros" 切换，因此无需在这里写 USER ros。