# ==============================================================
# 2026_Gsing-second_ROS — Docker Image
# ROS2 Jazzy + FAST-LIO2 SLAM + Nav2 + STM32 Serial Control
# ==============================================================
FROM ros:jazzy-ros-base AS builder

# ---------- 构建依赖 ----------
RUN apt update && apt install -y --no-install-recommends \
    python3-pip python3-colcon-common-extensions \
    cmake g++ libpcl-dev libeigen3-dev \
    ros-jazzy-nav2-bringup ros-jazzy-nav2-msgs \
    ros-jazzy-tf-transformations \
    ros-jazzy-robot-localization \
    ros-jazzy-pointcloud-to-laserscan \
    ros-jazzy-turtle-tf2-py \
    ros-jazzy-launch-xml \
    ros-jazzy-rviz2 \
    && rm -rf /var/lib/apt/lists/*

# ---------- Python 依赖 ----------
RUN pip3 install --no-cache-dir \
    open3d tf_transformations \
    transforms3d scipy numpy

# ---------- 构建 FAST-LIO2 工作空间 ----------
WORKDIR /ws_fastlio2
COPY fastlio2_v2/src ./src
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --symlink-install --packages-select \
        unitree_lidar_ros2 fast_lio pcd2pgm fast_lio_localization && \
    bash src/fast_lio_localization/scripts/hook_fix.sh

# ---------- 构建 Nav2 工作空间 ----------
WORKDIR /ws_nav2
COPY nav2_ws1/src ./src
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --symlink-install

# ---------- 运行时镜像 ----------
FROM ros:jazzy-ros-base

# 运行时依赖
RUN apt update && apt install -y --no-install-recommends \
    python3-pip python3-serial \
    ros-jazzy-nav2-bringup ros-jazzy-nav2-msgs \
    ros-jazzy-tf-transformations \
    ros-jazzy-robot-localization \
    ros-jazzy-pointcloud-to-laserscan \
    ros-jazzy-rviz2 \
    libpcl-dev libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir \
    open3d tf_transformations \
    transforms3d scipy numpy

# 从构建阶段复制编译产物
COPY --from=builder /ws_fastlio2 /ws_fastlio2
COPY --from=builder /ws_nav2 /ws_nav2

# 脚本工具
COPY py /app/py
WORKDIR /app

# hook_fix 脚本（启动时运行）
RUN echo '#!/bin/bash\n\
export AMENT_PREFIX_PATH="$AMENT_PREFIX_PATH:/ws_fastlio2/install/fast_lio_localization"\n\
exec "$@"' > /entrypoint.sh && chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

# 使用方式：
# 1. 构建：  docker build -t ros2-robot .
# 2. 运行：
#    docker run -it --rm \
#      --network=host \
#      --device=/dev/ttyACM0:/dev/ttyACM0 \
#      --device=/dev/video0:/dev/video0 \
#      -e DISPLAY=$DISPLAY \
#      -v /tmp/.X11-unix:/tmp/.X11-unix \
#      ros2-robot bash
# 3. 然后按照 hyper.md 的流程在新终端中逐一启动
