"""
launch_utils.py — 前置 ROS 节点启动/清理工具

供 auto_task.py 和 box_pick_node.py 共用，集中管理：
  - LiDAR 驱动 (unitree_lidar_ros2)
  - ICP 定位 (FAST-LIO2 + transform_fusion)
  - odometry→TF 桥
  - Nav2 导航
  - 串口桥 (cmd_vel_chassis_serial.py)

日志输出：所有子进程的 stdout/stderr 写入 logs/ 目录，按节点名+时间命名。
启动前自动杀残留进程，退出时自动清理。
"""

import atexit
import os
import platform
import signal
import subprocess
import sys
import time
from pathlib import Path

_PROJECT = Path(__file__).resolve().parent.parent.parent  # 2026_Gsing-second_ROS/
_LOG_DIR = _PROJECT / "logs"
_LOG_DIR.mkdir(parents=True, exist_ok=True)
os.environ["ROS_LOG_DIR"] = str(_LOG_DIR / "ros")
_PROCESSES = []

# ============ 退出/启动时杀的 ROS 进程模式列表 ============
_KILL_PATTERNS = [
    "unitree_lidar_ros2_node",
    "fastlio_mapping",
    "global_localization",
    "transform_fusion",
    "odometry_to_tf",
    "cmd_vel_chassis_serial",
    "controller_server",
    "planner_server",
    "bt_navigator",
    "lifecycle_manager",
    "rviz2",
]

# ============ 分类映射（name→子目录） ============
_LOG_CATEGORIES = {
    "LiDAR": "lidar",
    "ICP": "slam",
    "TF桥": "slam",
    "FAST-LIO2": "slam",
    "RViz": "nav2",
    "Nav2": "nav2",
    "串口桥": "serial",
    "腿部调试": "serial",
}

# ============ 开关 ============
# RViz 默认：miniPC(gsing) 不启动，主机(hyper) 启动
# 可通过环境变量 GSING_RVIZ=1 或 GSING_RVIZ=0 临时覆盖
_HOST_RVIZ_DEFAULT = "0" if platform.node().lower() == "gsing" else "1"
ENABLE_RVIZ = os.environ.get("GSING_RVIZ", _HOST_RVIZ_DEFAULT).lower() not in ("0", "false", "off")
USE_TERMINAL = True    # 是否用独立终端窗口显示每个节点输出
SERIAL_PORT = "/dev/ttyACM0"   # STM32 串口设备路径（被下方自动检测覆盖）
MAP_NAME = "map/PCD30"  # 地图文件名（不含扩展名），同时用于 PCD 和 YAML。标准比赛地图

# ============ 硬件自动检测 ============
try:
    sys.path.insert(0, str(_PROJECT / "py"))
    from tools.detect_hardware import detect_all
    _hw = detect_all(verbose=False)
    sys.path.pop(0)
    if _hw["serial_port"]:
        print(f"  [硬件] 自动检测到 STM32 串口: {_hw['serial_port']}")
        SERIAL_PORT = _hw["serial_port"]
    if _hw["lidar_iface"]:
        print(f"  [硬件] LiDAR 网卡: {_hw['lidar_iface']}  IP: {_hw['lidar_local_ip']}")
    if _hw["lidar_reachable"]:
        print(f"  [硬件] LiDAR {_hw['lidar_ip']} → 可达")
except Exception:
    pass


def _log_path(name):
    """生成日志文件路径: logs/{category}/YYYY-MM-DD_HHMMSS_name.log"""
    cat = _LOG_CATEGORIES.get(name, "other")
    (cat_dir := _LOG_DIR / cat).mkdir(parents=True, exist_ok=True)
    ts = time.strftime("%Y-%m-%d_%H%M%S")
    return str(cat_dir / f"{ts}_{name}.log")


def launch(cmd, cwd=None, name=""):
    """启动一个后台 ROS 进程（或终端窗口），输出写入 logs/"""
    env = os.environ.copy()
    env["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"

    logfile = _log_path(name)
    f = open(logfile, "w")

    if USE_TERMINAL:
        # 每个节点开一个 gnome-terminal 窗口，顺便写日志
        term_cmd = f"cd {cwd or '.'} && ({cmd}) 2>&1 | tee {logfile}; exec bash"
        p = subprocess.Popen(
            ["gnome-terminal", "--", "bash", "-c", term_cmd],
            env=env,
        )
        print(f"  [{name}] 终端窗口已启动 → {logfile}")
        f.close()
        return p

    p = subprocess.Popen(
        ["bash", "-c", cmd], cwd=cwd, env=env, preexec_fn=os.setsid,
        stdout=f, stderr=subprocess.STDOUT,
    )
    f.close()  # 子进程已继承 fd
    _PROCESSES.append(p)
    print(f"  [{name}] PID={p.pid} → {logfile}")
    return p


def _clean_cyclone_shm():
    """清理 CycloneDDS 共享内存（防止 participant 索引耗尽）"""
    import shutil
    for pat in ["*cyclone*", "*dds*"]:
        for p in Path("/dev/shm").glob(pat):
            try:
                if p.is_dir():
                    shutil.rmtree(p, ignore_errors=True)
                else:
                    p.unlink(missing_ok=True)
            except Exception:
                pass


def _kill_existing():
    """杀残留 ROS 进程（启动前 + 退出时均调用）"""
    for pattern in _KILL_PATTERNS:
        try:
            subprocess.run(["pkill", "-f", pattern], timeout=5, capture_output=True)
        except Exception:
            pass


def cleanup_all():
    """清理所有子进程"""
    for p in _PROCESSES:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGTERM)
        except Exception:
            pass
    _PROCESSES.clear()

    # 杀终端窗口内的 ROS 进程 + 清理 DDS 共享内存
    _kill_existing()
    _clean_cyclone_shm()


def _register_cleanup():
    """注册退出清理"""
    atexit.register(cleanup_all)
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: cleanup_all())


def start_prerequisites(map_pcd=None, map_yaml=None):
    """
    启动所有前置 ROS 节点。

    Args:
        map_pcd: PCD 地图路径（用于 ICP 定位），默认 MAP_NAME + ".pcd"
        map_yaml: YAML 地图路径（用于 Nav2），默认 MAP_NAME + ".yaml"
    """
    # 启动前杀残留进程 + 清理 DDS 共享内存
    _kill_existing()
    _clean_cyclone_shm()

    fastlio_dir = str(_PROJECT / "fastlio2_v2")
    nav2_dir = str(_PROJECT / "nav2_ws1")

    if map_pcd is None:
        map_pcd = str(_PROJECT / f"{MAP_NAME}.pcd")
    if map_yaml is None:
        map_yaml = str(_PROJECT / f"{MAP_NAME}.yaml")

    ros_setup = "source /opt/ros/jazzy/setup.bash"

    print("╔══════════════════════════════════════════════╗")
    print("║  启动前置 ROS 节点                            ║")
    print("╚══════════════════════════════════════════════╝")

    # LiDAR / ICP 始终不开 RViz（终端 + 日志）
    rviz_arg = "false"
    # Nav2 的 RViz 受 GSING_RVIZ 环境变量控制（默认 miniPC 关，主机开）
    nav2_rviz_arg = "true" if ENABLE_RVIZ else "false"

    # LiDAR 驱动
    launch(
        f"cd {fastlio_dir} && {ros_setup} && source install/setup.bash && "
        f"ros2 launch unitree_lidar_ros2 launch.py start_rviz:={rviz_arg}",
        name="LiDAR",
    )

    # ICP 定位 (FAST-LIO2 + transform_fusion)
    launch(
        f"cd {fastlio_dir} && {ros_setup} && source install/setup.bash && "
        f"export AMENT_PREFIX_PATH=\"$PWD/install/fast_lio_localization:$AMENT_PREFIX_PATH\" && "
        f"export PYTHONPATH=\"$PYTHONPATH:$HOME/.local/lib/python3.12/site-packages\" && "
        f"ros2 launch fast_lio_localization 1.launch.py "
        f"  map:={map_pcd} config_file:=unilidar_l2.yaml rviz:={rviz_arg} "
        f"  map_voxel_size:=0.008 scan_voxel_size:=0.02 "
        f"  freq_localization:=2.0 localization_threshold:=0.9",
        name="ICP",
    )

    # odometry→TF 桥
    odom_bin = f"{fastlio_dir}/build/fast_lio/odometry_to_tf"
    if os.path.isfile(odom_bin):
        launch(f"cd {fastlio_dir} && {ros_setup} && source install/setup.bash && {odom_bin}", name="TF桥")
    else:
        print(f"  [TF桥] 未找到 {odom_bin}，跳过")

    # Nav2（按 GSING_RVIZ 决定是否开 RViz）
    launch(
        f"cd {nav2_dir} && {ros_setup} && source install/setup.bash && "
        f"ros2 launch dog_nav2_bringup nav2_fastlio_static_map.launch.py "
        f"  map:={map_yaml} start_rviz:={nav2_rviz_arg}",
        name="Nav2",
    )

    # 串口桥（串口设备不存在则跳过，避免进程崩溃）
    if os.path.exists(SERIAL_PORT):
        _LOG_DIR.mkdir(parents=True, exist_ok=True)
        (_LOG_DIR / "serial").mkdir(parents=True, exist_ok=True)
        leg_csv = str(_LOG_DIR / "serial" / f"{time.strftime('%Y-%m-%d_%H%M%S')}_leg.csv")
        launch(
            f"cd {nav2_dir} && {ros_setup} && source install/setup.bash && "
            f"ros2 launch dog_nav2_bringup chassis_serial_bridge.launch.py "
            f"  serial_port:={SERIAL_PORT} baud_rate:=115200 "
            f"  cmd_vel_topic:=/cmd_vel send_rate_hz:=50.0 "
            f"  active_state:=1 idle_state:=0 "
            f"  leg_debug_csv_path:={leg_csv} "
            f"  leg_debug_log_period_sec:=0.5",
            name="串口桥",
        )
    else:
        print(f"  [串口桥] {SERIAL_PORT} 不存在，跳过（插上 STM32 后手动启动）")

    # 注册清理
    _register_cleanup()
    print()
