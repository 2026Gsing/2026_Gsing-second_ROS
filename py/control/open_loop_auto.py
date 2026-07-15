#!/usr/bin/env python3
"""
open_loop_auto.py — 开环自动控制脚本（无 Nav2 定位）

流程:
  1. 开门禁 (AUTO_CMD_START)
  2. 前进 3s → 接近物资箱
  3. 启动 cube_detector.py + catch.py 检测并抓取
  4. 前进 6s → 携带物资箱到放置区
  5. 放下物资箱（按抓取位置）
  6. 完成

与 auto_task.py 的区别：
  - 不使用 Nav2 导航，全靠开环速度控制
  - 不需要地图，不需要 ICP 定位
  - 只需要 LiDAR + 串口桥

使用方式:
  ./ros-run.sh py/control/open_loop_auto.py
  ./ros-run.sh py/control/open_loop_auto.py --no-startup   # 跳过前置节点启动
  ./ros-run.sh py/control/open_loop_auto.py --vx 0.4       # 更快前进
  ./ros-run.sh py/control/open_loop_auto.py --approach 5.0 --carry 8.0  # 自定义时长
"""

import argparse
import json
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from visualization_msgs.msg import Marker

_HERE = Path(__file__).resolve().parent
_PROJECT = _HERE.parent.parent

sys.path.insert(0, str(_HERE))
from launch_utils import launch, cleanup_all, _kill_existing, _clean_cyclone_shm

# ============================================================
# 参数（可被 --vx / --approach / --carry 覆盖）
# ============================================================
FORWARD_SPEED = 0.2        # 前进速度 (m/s)
APPROACH_TIME = 10.0        # 第一次前进持续时间 (s)
CARRY_TIME = 16.0           # 第二次前进持续时间 (s)
GRAB_TIMEOUT = 20.0        # 抓取超时 (s)
PLACE_TIMEOUT = 20.0       # 放置超时 (s)

# ============================================================
# 串口协议常量（与 STM32 / catch.py / auto_task.py 一致）
# ============================================================
AUTO_CMD_START = 1
AUTO_CMD_ARRIVED_BOX = 2
AUTO_CMD_ARRIVED_ZONE = 4
AUTO_CMD_FINISH = 7

ARM_MISSION_BACK_TO_PLACE = 2
ARM_MISSION_HAS_PLACE = 0x04

# 放置坐标（机械臂系）
# arm_x(高度), arm_y(侧向), arm_z(前向)
PLACE_X = -0.25   # 向下放
PLACE_Y = 0.0     # 正中
PLACE_Z = -0.35   # 臂向前伸


class OpenLoopAutoNode(Node):
    """开环自动控制节点"""

    def __init__(self):
        super().__init__("open_loop_auto")

        # ==================== 发布器 ====================
        self.vel_pub = self.create_publisher(Twist, "/vision_cmd_vel", 10)
        self.auto_pub = self.create_publisher(String, "/vision/auto_cmd", 10)
        self.arm_pub = self.create_publisher(String, "/vision/arm_mission", 10)

        # ==================== 订阅 cube 检测（仅日志） ====================
        self._detected_cube = None
        self.create_subscription(Marker, "/detected_cube", self._cube_cb, 10)

        # ==================== 50Hz 速度重发机制 ====================
        # 防止 STM32 串口桥的 500ms 视觉超时自动停车
        self._vx = 0.0
        self._wz = 0.0
        self.create_timer(0.02, self._republish_cb)

        # ==================== 子进程 ====================
        self._cube_proc = None
        self._catch_proc = None
        self._subprocess_lock = threading.Lock()

        # ==================== 后台 spin 线程 ====================
        threading.Thread(target=self._bg_spin, daemon=True).start()

        # ==================== 启动前置节点 ====================
        # 不再在 __init__ 中启动，由 main() 调用 standalone startup()

        self.get_logger().info("=" * 60)
        self.get_logger().info("  开环自动控制脚本")
        self.get_logger().info(f"  流程: 开门 → 前进 {APPROACH_TIME}s → 抓取 "
                               f"→ 前进 {CARRY_TIME}s → 放置")
        self.get_logger().info("=" * 60)

    # ──────────────────────────────────────────────────────────────
    # 回调
    # ──────────────────────────────────────────────────────────────

    def _cube_cb(self, msg):
        """记录检测到的立方体（用于日志显示）"""
        self._detected_cube = msg

    def _bg_spin(self):
        """后台 spin 线程"""
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

    def _republish_cb(self):
        """50Hz 持续重发当前目标速度，防止 500ms 视觉超时停车"""
        msg = Twist()
        msg.linear.x = self._vx
        msg.angular.z = self._wz
        self.vel_pub.publish(msg)

    # ──────────────────────────────────────────────────────────────
    # 速度控制
    # ──────────────────────────────────────────────────────────────

    def set_vel(self, vx, wz=0.0):
        """设置目标速度（定时器以 50Hz 持续重发）"""
        self._vx = float(vx)
        self._wz = float(wz)
        # 立即发一次，避免等待定时器下次触发
        msg = Twist()
        msg.linear.x = self._vx
        msg.angular.z = self._wz
        self.vel_pub.publish(msg)
        self.get_logger().info(f"[VEL] vx={vx:+.2f}  wz={wz:+.2f}")

    def stop(self):
        """停止运动"""
        self.set_vel(0.0, 0.0)
        self.get_logger().info("[VEL] 停止")

    def move_open_loop(self, duration, speed=None):
        """
        开环前进指定时长。

        在移动过程中持续 spin 以保持回调处理。
        """
        if speed is None:
            speed = FORWARD_SPEED
        self.get_logger().info(f"[MOVE] 前进 {duration:.1f}s @ {speed:.2f}m/s")
        self.set_vel(speed, 0.0)
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
        self.stop()

    # ──────────────────────────────────────────────────────────────
    # 串口指令
    # ──────────────────────────────────────────────────────────────

    def send_auto_cmd(self, cmd, target=0, zone=0):
        """发送 0x15 自动任务命令（通过串口桥）"""
        payload = json.dumps({"cmd": cmd, "target": target, "zone": zone})
        self.auto_pub.publish(String(data=payload))
        cmd_names = {1: "START", 2: "ARRIVED_BOX", 4: "ARRIVED_ZONE", 7: "FINISH"}
        name = cmd_names.get(cmd, f"0x{cmd:02x}")
        self.get_logger().info(f"[AUTO] {name} cmd={cmd} target={target} zone={zone}")

    def send_arm_mission(self, mode, flags, sequences):
        """
        发送 0x14 机械臂任务（通过串口桥）。

        Args:
            mode: 任务模式 (1=PICK_TO_BACK, 2=BACK_TO_PLACE)
            flags: 位标志 (0x01=HAS_PICK, 0x02=HAS_BACK, 0x04=HAS_PLACE)
            sequences: dict 含 pick/back/place 坐标
        """
        payload = {"mode": mode, "flags": flags}
        for key in ("pick", "back", "place"):
            if key in sequences:
                payload[key] = sequences[key]
        self.arm_pub.publish(String(data=json.dumps(payload)))
        mode_names = {1: "PICK_TO_BACK", 2: "BACK_TO_PLACE"}
        name = mode_names.get(mode, f"mode={mode}")
        self.get_logger().info(f"[ARM] {name} flags={flags}")

    # ──────────────────────────────────────────────────────────────
    # 子进程管理（与 auto_task.py 一致）
    # ──────────────────────────────────────────────────────────────

    def _start_cube_detector(self):
        """启动 cube_detector.py 子进程"""
        with self._subprocess_lock:
            if self._cube_proc is not None:
                return
            script = str(_HERE / "cube_detector.py")
            logdir = _PROJECT / "logs" / "vision"
            logdir.mkdir(parents=True, exist_ok=True)
            logfile = str(logdir / f"{time.strftime('%Y-%m-%d_%H%M%S')}_cube_detector.log")
            try:
                f = open(logfile, "w")
                self._cube_proc = subprocess.Popen(
                    [sys.executable, script],
                    stdout=f, stderr=subprocess.STDOUT,
                )
                f.close()
                self.get_logger().info(f"[CUBE] PID={self._cube_proc.pid} → {logfile}")
            except Exception as e:
                self.get_logger().error(f"[CUBE] 启动失败: {e}")

    def _start_catch(self):
        """启动 catch.py 子进程"""
        with self._subprocess_lock:
            if self._catch_proc is not None:
                return
            script = str(_HERE / "catch.py")
            logdir = _PROJECT / "logs" / "arm"
            logdir.mkdir(parents=True, exist_ok=True)
            logfile = str(logdir / f"{time.strftime('%Y-%m-%d_%H%M%S')}_catch.log")
            try:
                f = open(logfile, "w")
                self._catch_proc = subprocess.Popen(
                    [sys.executable, script],
                    stdout=f, stderr=subprocess.STDOUT,
                )
                f.close()
                self.get_logger().info(f"[CATCH] PID={self._catch_proc.pid} → {logfile}")
            except Exception as e:
                self.get_logger().error(f"[CATCH] 启动失败: {e}")

    def _cleanup_subprocesses(self):
        """停止 cube_detector 和 catch 子进程"""
        with self._subprocess_lock:
            for attr in ("_cube_proc", "_catch_proc"):
                proc = getattr(self, attr, None)
                if proc is None:
                    continue
                try:
                    proc.send_signal(signal.SIGINT)
                    proc.wait(timeout=3.0)
                except Exception:
                    try:
                        proc.kill()
                        proc.wait()
                    except Exception:
                        pass
                setattr(self, attr, None)

    # ──────────────────────────────────────────────────────────────
    # 主流程步骤
    # ──────────────────────────────────────────────────────────────

    def step_open_gate(self):
        """[步骤 1] 开门禁 — 发送 START 允许底盘运动"""
        self.get_logger().info("")
        self.get_logger().info("╔══════════════════════════════════════╗")
        self.get_logger().info("║  步骤 1/5: 开门禁 (AUTO_CMD_START)  ║")
        self.get_logger().info("╚══════════════════════════════════════╝")
        self.send_auto_cmd(AUTO_CMD_START)
        time.sleep(0.3)
        self.get_logger().info("[OK] 门禁已开")

    def step_approach(self):
        """[步骤 2] 前进指定时长接近物资箱"""
        self.get_logger().info("")
        self.get_logger().info("╔══════════════════════════════════════╗")
        self.get_logger().info(f"║  步骤 2/5: 前进 {APPROACH_TIME}s 接近物资箱  ║")
        self.get_logger().info("╚══════════════════════════════════════╝")
        self.move_open_loop(APPROACH_TIME)
        self.get_logger().info("[OK] 到达物资箱区域")

    def step_grab(self):
        """[步骤 3] 启动 cube_detector + catch 检测并抓取"""
        self.get_logger().info("")
        self.get_logger().info("╔══════════════════════════════════════╗")
        self.get_logger().info("║  步骤 3/5: 检测并抓取物资箱         ║")
        self.get_logger().info("╚══════════════════════════════════════╝")

        # ── 启动 cube_detector（先启动，让它积累点云帧） ──
        self._start_cube_detector()
        self.get_logger().info("[抓取] 等待 cube_detector 初始化 (1.5s)...")
        time.sleep(1.5)

        # ── 启动 catch.py ──
        # catch.py 内部完整流程：
        #   订阅 /detected_cube → 稳定检测（滑动窗口标准差）
        #   → 推进 STM32 状态机 (START → ARRIVED_BOX → PICK)
        #   → 发 0x14 PICK_TO_BACK 机械臂坐标
        #   → 等待 STM32 回传 ARM_EVENT pick_done 后退出
        # 不需要外部预推状态机，完全由 catch.py 等检测稳定后再操作
        self._start_catch()

        # ── 等待抓取完成（catch.py 等到 pick_done 才退出） ──
        if self._catch_proc:
            self.get_logger().info(
                f"[抓取] 等待 STM32 抓取完成 (PID={self._catch_proc.pid}) "
                f"超时 {GRAB_TIMEOUT:.0f}s..."
            )
            try:
                self._catch_proc.wait(timeout=GRAB_TIMEOUT)
                self.get_logger().info("[抓取] ✅ STM32 抓取完成")
            except subprocess.TimeoutExpired:
                self.get_logger().warn(f"[抓取] ⏰ STM32 抓取超时 ({GRAB_TIMEOUT:.0f}s)")
                try:
                    self._catch_proc.kill()
                    self._catch_proc.wait(timeout=3.0)
                except Exception:
                    pass

        # ── 清理 ──
        self._cleanup_subprocesses()
        self.get_logger().info("[抓取] ✅ 抓取阶段完成，继续前进")

    def step_carry(self):
        """[步骤 4] 前进指定时长携带物资箱到放置区"""
        self.get_logger().info("")
        self.get_logger().info("╔══════════════════════════════════════╗")
        self.get_logger().info(f"║  步骤 4/5: 前进 {CARRY_TIME}s 携带到放置区     ║")
        self.get_logger().info("╚══════════════════════════════════════╝")
        self.move_open_loop(CARRY_TIME)
        self.get_logger().info("[OK] 到达放置区")

    def step_place(self):
        """[步骤 5] 放下物资箱"""
        self.get_logger().info("")
        self.get_logger().info("╔══════════════════════════════════════╗")
        self.get_logger().info("║  步骤 5/5: 放下物资箱               ║")
        self.get_logger().info("╚══════════════════════════════════════╝")

        # 推进 STM32 状态机到 PLACE
        # 根据抓取时的箱子坐标选择归位区（有记录则用，无则用 zone=0）
        zone_id = 0
        if self._detected_cube is not None:
            cx = self._detected_cube.pose.position.x
            # 箱子在雷达系中的 x 坐标 ≈ 离机器人距离
            # 距离远 (x > 0.15) 表示箱子在前方区域，归位区 0
            # 可在此处根据实际情况调整 zone 选择逻辑
            zone_id = 0
            self.get_logger().info(
                f"[放置] 抓取位置: x={cx:.3f} (雷达系) → 归位区 {zone_id}"
            )
        else:
            self.get_logger().info("[放置] 无检测记录，默认归位区 0")

        self.send_auto_cmd(AUTO_CMD_ARRIVED_ZONE, target=1, zone=zone_id)
        time.sleep(0.5)

        # 发送放置坐标（与 auto_task.py 一致）
        self.send_arm_mission(
            mode=ARM_MISSION_BACK_TO_PLACE,
            flags=ARM_MISSION_HAS_PLACE,
            sequences={"place": {"x": PLACE_X, "y": PLACE_Y, "z": PLACE_Z}},
        )

        # 等待 STM32 执行放置
        self.get_logger().info(f"[放置] 等待机械臂执行放置（超时 {PLACE_TIMEOUT:.0f}s）...")
        time.sleep(PLACE_TIMEOUT)
        self.get_logger().info("[放置] ✅ 放置完成")

    def finish(self):
        """完成 — 发送 FINISH + 停止 + 清理"""
        self.get_logger().info("")
        self.get_logger().info("╔══════════════════════════════════════╗")
        self.get_logger().info("║  开环自动控制完成 ✅                ║")
        self.get_logger().info("╚══════════════════════════════════════╝")
        self.send_auto_cmd(AUTO_CMD_FINISH)
        self.stop()
        self._cleanup_subprocesses()


# ============================================================
# 启动前置节点（参照 start_prerequisites 的写法）
# ============================================================

def _startup():
    """启动 LiDAR 驱动 + 串口桥（开环控制不需要 ICP/Nav2）"""
    # ── 启动日志文件 ──
    _startup_logdir = _PROJECT / "logs" / "startup"
    _startup_logdir.mkdir(parents=True, exist_ok=True)
    _startup_log = _startup_logdir / f"{time.strftime('%Y-%m-%d_%H%M%S')}_open_loop_startup.log"

    def _log(msg):
        print(msg)
        with open(_startup_log, "a") as f:
            f.write(msg + "\n")

    _log("[启动] 启动前置节点 (LiDAR + 串口桥)...")
    _log(f"[启动] 日志 → {_startup_log}")

    _kill_existing()
    _clean_cyclone_shm()

    # 硬件自动检测
    _hw = {}
    try:
        sys.path.insert(0, str(_PROJECT / "py"))
        from tools.detect_hardware import detect_all
        _hw = detect_all(verbose=False)
        sys.path.pop(0)
        _log(f"[硬件] 串口={_hw.get('serial_port','?')}  LiDAR IP={_hw.get('lidar_ip','?')}")
    except Exception as e:
        _log(f"[硬件] 自动检测失败: {e}")

    _lidar_ip = _hw.get("lidar_ip", "192.168.1.1")
    _local_ip = _hw.get("local_ip", "192.168.1.2")
    _serial_port = _hw.get("serial_port", "/dev/ttyACM0")

    # 检测 ROS2 发行版
    _ros_setup = None
    for _d in ("jazzy", "humble"):
        _p = f"/opt/ros/{_d}/setup.bash"
        if os.path.isfile(_p):
            _ros_setup = f"source {_p}"
            _log(f"[启动] ROS2 发行版: {_d}")
            break
    if _ros_setup is None:
        _ros_setup = "source /opt/ros/humble/setup.bash"
        _log("[启动] ⚠️ 未检测到 ROS2，默认 humble")

    _fastlio_dir = str(_PROJECT / "fastlio2_v2")
    _nav2_dir = str(_PROJECT / "nav2_ws1")

    # ── LiDAR 驱动 ──
    _log("[启动] 启动 LiDAR 驱动...")
    launch(
        f"cd {_fastlio_dir} && {_ros_setup} && source install/setup.bash && "
        f"ros2 launch unitree_lidar_ros2 launch.py start_rviz:=false "
        f"lidar_ip:={_lidar_ip} local_ip:={_local_ip}",
        name="LiDAR",
    )

    # 等待 LiDAR 初始化 + 话题就绪
    _log("[启动] 等待 10s 让 LiDAR 驱动就绪...")
    time.sleep(10)

    # 验证 LiDAR 话题（同 start_prerequisites）
    try:
        result = subprocess.run(
            ["bash", "-c", f"{_ros_setup} && ros2 topic list 2>/dev/null | grep -q unilidar/cloud"],
            timeout=3, capture_output=True,
        )
        if result.returncode == 0:
            _log("[启动] ✅ LiDAR 话题已就绪")
        else:
            _log("[启动] ⚠️ LiDAR 话题未检测到（但仍继续）")
    except Exception:
        _log("[启动] ⚠️ 话题检测失败（但仍继续）")

    # ── 串口桥 ──
    if os.path.exists(_serial_port):
        _log(f"[启动] 启动串口桥 ({_serial_port})...")
        _serial_logdir = _PROJECT / "logs" / "serial"
        _serial_logdir.mkdir(parents=True, exist_ok=True)
        _leg_csv = str(_serial_logdir / f"{time.strftime('%Y-%m-%d_%H%M%S')}_leg.csv")
        launch(
            f"cd {_nav2_dir} && {_ros_setup} && source install/setup.bash && "
            f"ros2 launch dog_nav2_bringup chassis_serial_bridge.launch.py "
            f"  serial_port:={_serial_port} baud_rate:=115200 "
            f"  cmd_vel_topic:=/cmd_vel send_rate_hz:=50.0 "
            f"  active_state:=1 idle_state:=0 "
            f"  leg_debug_csv_path:={_leg_csv} "
            f"  leg_debug_log_period_sec:=0.5",
            name="串口桥",
        )
    else:
        _log(f"[启动] {_serial_port} 不存在，跳过串口桥")

    _log("[启动] ✅ 前置节点启动完成")


# ============================================================
# SIGINT 处理器（同 auto_task.py）
# ============================================================
def _sigint_handler(sig, frame):
    print("\n[退出] 收到 Ctrl+C，正在关闭...")
    cleanup_all()
    rclpy.shutdown()


# ============================================================
# 入口
# ============================================================

def main():
    signal.signal(signal.SIGINT, _sigint_handler)
    os.environ.setdefault("ROS_LOG_DIR", str(_PROJECT / "logs" / "ros"))

    global FORWARD_SPEED, APPROACH_TIME, CARRY_TIME

    parser = argparse.ArgumentParser(description="开环自动控制脚本")
    parser.add_argument("--no-startup", action="store_true",
                        help="不启动前置节点（LiDAR/串口桥需已运行）")
    parser.add_argument("--vx", "--speed", type=float, default=FORWARD_SPEED,
                        help=f"前进速度 m/s (默认 {FORWARD_SPEED})")
    parser.add_argument("--approach", type=float, default=None,
                        help=f"第一次前进时长 s (默认 {APPROACH_TIME})")
    parser.add_argument("--carry", type=float, default=None,
                        help=f"第二次前进时长 s (默认 {CARRY_TIME})")
    args = parser.parse_args()

    # 通过全局变量透传 CLI 参数
    FORWARD_SPEED = args.vx
    if args.approach is not None:
        APPROACH_TIME = args.approach
    if args.carry is not None:
        CARRY_TIME = args.carry

    print("╔══════════════════════════════════════════════╗")
    print("║  开环自动控制                               ║")
    print("║  流程: 开门 → 前进 → 抓取 → 前进 → 放置    ║")
    print("╚══════════════════════════════════════════════╝")

    # ── 启动前置节点（LiDAR + 串口桥），不含 ICP/Nav2 ──
    if not args.no_startup:
        _startup()

    # ── ROS 初始化 ──
    rclpy.init()
    node = OpenLoopAutoNode()

    # ── 等待 DDS 发现完成 ──
    print(f"\n[启动] 等待 5s 让 ROS 节点就绪...")
    time.sleep(5)

    # ── 执行各步骤 ──
    try:
        node.step_open_gate()
        node.step_approach()

        t0 = time.monotonic()
        node.step_grab()
        t_grab = time.monotonic() - t0
        node.get_logger().info(f"[耗时] 抓取阶段用时 {t_grab:.1f}s")

        time.sleep(1.0)  # 短暂停让机械臂稳定

        node.step_carry()
        node.step_place()
        node.finish()

    except KeyboardInterrupt:
        node.get_logger().info("\n[中断] 收到 Ctrl+C")
        node.stop()
        node.send_auto_cmd(AUTO_CMD_FINISH)
        node._cleanup_subprocesses()
    except Exception as e:
        node.get_logger().error(f"[错误] {e}")
        import traceback
        node.get_logger().error(traceback.format_exc())
        node.stop()
        node._cleanup_subprocesses()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        cleanup_all()


if __name__ == "__main__":
    main()
