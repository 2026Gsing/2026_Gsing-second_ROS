"""
launch_utils.py — 前置 ROS 节点启动/清理工具

供 auto_task.py 和 box_pick_node.py 共用，集中管理：
  - LiDAR 驱动 (unitree_lidar_ros2)
  - ICP 定位 (FAST-LIO2 + transform_fusion)
  - odometry→TF 桥
  - Nav2 导航
  - 串口桥 (cmd_vel_chassis_serial.py)
"""

import atexit
import os
import signal
import subprocess
from pathlib import Path

_PROJECT = Path(__file__).resolve().parent.parent.parent  # 2026_Gsing-second_ROS/
_PROCESSES = []

# ============ 开关 ============
ENABLE_RVIZ = True     # ICP 定位启动时是否打开 RViz 可视化
USE_TERMINAL = True    # 是否用独立终端窗口显示每个节点输出


def launch(cmd, cwd=None, name=""):
    """启动一个后台 ROS 进程（或终端窗口）"""
    env = os.environ.copy()
    env["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"

    if USE_TERMINAL:
        # 每个节点开一个 gnome-terminal 窗口，方便看输出调试
        term_cmd = f"cd {cwd or '.'} && {cmd}; exec bash"
        p = subprocess.Popen(
            ["gnome-terminal", "--", "bash", "-c", term_cmd],
            env=env,
        )
        print(f"  [{name}] 终端窗口已启动")
        return p

    p = subprocess.Popen(
        ["bash", "-c", cmd], cwd=cwd, env=env, preexec_fn=os.setsid,
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )
    _PROCESSES.append(p)
    print(f"  [{name}] PID={p.pid}")
    return p


def cleanup_all():
    """清理所有子进程"""
    for p in _PROCESSES:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGTERM)
        except Exception:
            pass
    _PROCESSES.clear()


def _register_cleanup():
    """注册退出清理"""
    atexit.register(cleanup_all)
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: cleanup_all())


def start_prerequisites(map_pcd=None, map_yaml=None):
    """
    启动所有前置 ROS 节点。

    Args:
        map_pcd: PCD 地图路径（用于 ICP 定位），默认 map/map.pcd
        map_yaml: YAML 地图路径（用于 Nav2），默认 map/map.yaml
    """
    fastlio_dir = str(_PROJECT / "fastlio2_v2")
    nav2_dir = str(_PROJECT / "nav2_ws1")
    map_dir = _PROJECT / "map"

    if map_pcd is None:
        map_pcd = str(map_dir / "map.pcd")
    if map_yaml is None:
        map_yaml = str(map_dir / "map.yaml")

    ros_setup = "source /opt/ros/jazzy/setup.bash"

    print("╔══════════════════════════════════════════════╗")
    print("║  启动前置 ROS 节点                            ║")
    print("╚══════════════════════════════════════════════╝")

    # LiDAR 驱动
    launch(
        f"cd {fastlio_dir} && {ros_setup} && source install/setup.bash && "
        f"ros2 launch unitree_lidar_ros2 launch.py",
        name="LiDAR",
    )

    rviz_arg = "true" if ENABLE_RVIZ else "false"

    # ICP 定位 (FAST-LIO2 + transform_fusion)
    launch(
        f"cd {fastlio_dir} && {ros_setup} && source install/setup.bash && "
        f"export AMENT_PREFIX_PATH=\"$PWD/install/fast_lio_localization:$AMENT_PREFIX_PATH\" && "
        f"export PYTHONPATH=\"$PYTHONPATH:$HOME/.local/lib/python3.12/site-packages\" && "
        f"ros2 launch fast_lio_localization 1.launch.py "
        f"  map:={map_pcd} config_file:=unilidar_l2.yaml rviz:={rviz_arg} "
        f"  map_voxel_size:=0.01 scan_voxel_size:=0.03 "
        f"  freq_localization:=2.0 localization_threshold:=0.9",
        name="ICP",
    )

    # odometry→TF 桥
    odom_bin = f"{fastlio_dir}/build/fast_lio/odometry_to_tf"
    if os.path.isfile(odom_bin):
        launch(f"cd {fastlio_dir} && {ros_setup} && source install/setup.bash && {odom_bin}", name="TF桥")
    else:
        print(f"  [TF桥] 未找到 {odom_bin}，跳过")

    # Nav2
    launch(
        f"cd {nav2_dir} && {ros_setup} && source install/setup.bash && "
        f"ros2 launch dog_nav2_bringup nav2_fastlio_static_map.launch.py "
        f"  map:={map_yaml} start_rviz:={rviz_arg}",
        name="Nav2",
    )

    # 串口桥
    launch(
        f"cd {nav2_dir} && {ros_setup} && source install/setup.bash && "
        f"ros2 launch dog_nav2_bringup chassis_serial_bridge.launch.py "
        f"  serial_port:=/dev/ttyACM0 baud_rate:=115200 "
        f"  cmd_vel_topic:=/cmd_vel send_rate_hz:=50.0 "
        f"  active_state:=1 idle_state:=0",
        name="串口桥",
    )

    # 注册清理
    _register_cleanup()
    print()
