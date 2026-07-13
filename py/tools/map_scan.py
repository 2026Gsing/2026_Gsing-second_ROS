#!/usr/bin/env python3
"""
map_scan.py — 一键建图脚本

流程：
  1. 启动 LiDAR 驱动（后台）
  2. 启动 FAST-LIO2 SLAM 建图（后台，按编号自动命名 PCD）
  3. 可选启动 RViz 可视化
  4. 按 Enter 保存地图并退出所有进程
  5. 自动将最新 PCD 转为 PGM 栅格地图

使用方式：
  python3 py/tools/map_scan.py          # 默认带 RViz
  python3 py/tools/map_scan.py --no-rviz   # 不带 RViz

前提：
  已 colcon build fastlio2_v2 工作空间
"""

import subprocess
import os
import signal
import sys
import time
from pathlib import Path

_HERE = Path(__file__).resolve().parent          # py/tools/
_PROJECT = _HERE.parent.parent                   # 2026_Gsing-second_ROS
_LOG_DIR = _PROJECT / "logs"
_FASTLIO_DIR = str(_PROJECT / "fastlio2_v2")
_MAP_DIR = str(_PROJECT / "map")
_ROS_SETUP = "source /opt/ros/jazzy/setup.bash"

_PROCS = []  # 子进程列表


def launch(cmd, name=""):
    """启动后台进程，输出写入 logs/"""
    env = os.environ.copy()
    env["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
    _LOG_DIR.mkdir(parents=True, exist_ok=True)
    logfile = _LOG_DIR / f"{time.strftime('%Y-%m-%d_%H%M%S')}_{name}.log"
    f = open(logfile, "w")
    p = subprocess.Popen(
        ["bash", "-c", cmd],
        preexec_fn=os.setsid,
        env=env,
        stdout=f, stderr=subprocess.STDOUT,
    )
    _PROCS.append(p)
    print(f"  [{name}] PID={p.pid} → {logfile}")
    return p


def cleanup():
    """清理所有子进程"""
    for p in _PROCS:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGTERM)
        except Exception:
            pass
    _PROCS.clear()


def find_next_pcd():
    """找到下一个可用的 PCD 编号"""
    os.makedirs(_MAP_DIR, exist_ok=True)
    next_num = 1
    for f in Path(_MAP_DIR).glob("PCD*.pcd"):
        try:
            num = int(f.stem[3:])
            if num >= next_num:
                next_num = num + 1
        except ValueError:
            pass
    return next_num, str(Path(_MAP_DIR) / f"PCD{next_num}.pcd")


def main():
    use_rviz = "--no-rviz" not in sys.argv

    print("=" * 50)
    print("  一键建图")
    print("=" * 50)
    print()

    # 自动编号
    pcd_num, pcd_path = find_next_pcd()
    print(f"  本次保存: {pcd_path}")
    print()

    # === 终端 1: LiDAR 驱动 ===
    print("[1/3] 启动 LiDAR 驱动...")
    launch(
        f"cd {_FASTLIO_DIR} && {_ROS_SETUP} && source install/setup.bash && "
        f"ros2 launch unitree_lidar_ros2 launch.py",
        name="LiDAR",
    )
    time.sleep(2)

    # === 终端 2: FAST-LIO2 SLAM 建图（禁用 IMU，以编号为保存路径）===
    print("[2/3] 启动 FAST-LIO2 SLAM 建图...")
    launch(
        f"cd {_FASTLIO_DIR} && {_ROS_SETUP} && source install/setup.bash && "
        f"python3 {_HERE}/pointcloud_x_filter.py & "
        f"ros2 run fast_lio fastlio_mapping --ros-args "
        f"  --params-file src/unilidar_fastlio_ros2-ros2/config/unilidar_l2.yaml "
        f"  -p \"common.lid_topic:=/unilidar/cloud_filtered\" "
        f"  -r /livox/imu:=/unilidar/imu "
        f"  -p imu_en:=false "
        f"  -p map_file_path:={pcd_path}",
        name="FAST-LIO2",
    )

    # === 终端 2b: RViz（可选）===
    if use_rviz:
        print("  [可选] 启动 RViz 可视化...")
        rviz_cfg = f"{_PROJECT}/fastlio2_v2/src/fast_lio_config.rviz"
        if os.path.isfile(rviz_cfg):
            launch(
                f"cd {_FASTLIO_DIR} && {_ROS_SETUP} && source install/setup.bash && "
                f"rviz2 -d {rviz_cfg}",
                name="RViz",
            )
        else:
            print(f"  [RViz] 配置文件不存在: {rviz_cfg}，跳过")

    print()
    print("=" * 50)
    print("  建图中… 推着机器人走一圈覆盖全场")
    print("  按 Enter 保存地图并退出")
    print("=" * 50)

    # 等待用户按回车
    try:
        input()
    except (EOFError, KeyboardInterrupt):
        pass

    print()
    print("=" * 50)
    print(f"  保存地图 → {pcd_path} ...")
    print("=" * 50)

    # 调用 /map_save 服务保存
    result = subprocess.run(
        ["bash", "-c",
         f"{_ROS_SETUP} && "
         f"ros2 service call /map_save std_srvs/srv/Trigger"],
        env={**os.environ, "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp"},
        capture_output=True, text=True, timeout=10,
    )
    print(f"  map_save 返回码={result.returncode}")
    if result.returncode != 0:
        print(f"  stderr: {result.stderr[:200]}")
    time.sleep(1)

    # 清理建图进程
    cleanup()
    print("  建图进程已停止")

    # 检查 PCD 是否有效
    pcd_size = os.path.getsize(pcd_path) if os.path.isfile(pcd_path) else 0
    print(f"  PCD 文件大小: {pcd_size} bytes")
    if pcd_size < 100:
        print("  ⚠ PCD 文件过小，可能是 LiDAR 未连接或无数据")

    # === 终端 3: PCD → PGM 转换 ===
    print()
    print("[3/3] 转换 PCD → PGM 栅格地图...")
    result2 = subprocess.run(
        ["bash", "-c",
         f"cd {_FASTLIO_DIR} && {_ROS_SETUP} && source install/setup.bash && "
         f"ros2 launch pcd2pgm pcd2pgm_launch.py "
         f"  pcd_file:={pcd_path}"],
        env={**os.environ, "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp"},
        capture_output=True, text=True, timeout=60,
    )
    print(f"  pcd2pgm 返回码={result2.returncode}")
    if result2.returncode != 0:
        print(f"  stderr: {result2.stderr[:500]}")
    else:
        print(f"  PGM 已保存到 {_MAP_DIR}/")
    print()
    print("=" * 50)
    print("  建图完成！")
    print(f"  PCD: {pcd_path}")
    if os.path.isfile(str(Path(_MAP_DIR) / f'PCD{pcd_num}.yaml')):
        print(f"  PGM: {Path(_MAP_DIR) / f'PCD{pcd_num}.yaml'}")
    print("=" * 50)


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"错误: {e}")
    finally:
        cleanup()
