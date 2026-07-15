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
from launch_utils import start_prerequisites, cleanup_all

# ============================================================
# 参数（可被 --vx / --approach / --carry 覆盖）
# ============================================================
FORWARD_SPEED = 0.2        # 前进速度 (m/s)
APPROACH_TIME = 3.0        # 第一次前进持续时间 (s)
CARRY_TIME = 6.0           # 第二次前进持续时间 (s)
GRAB_TIMEOUT = 35.0        # 抓取超时 (s)
PLACE_TIMEOUT = 25.0       # 放置超时 (s)

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

    def __init__(self, no_startup=False):
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
        if not no_startup:
            self._startup()

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
    # 启动前置节点
    # ──────────────────────────────────────────────────────────────

    def _startup(self):
        """
        启动 LiDAR + 串口桥（使用 launch_utils 的前置启动）。
        start_prerequisites 会启动 LiDAR、ICP 定位、TF 桥、Nav2、串口桥。
        虽然 ICP 和 Nav2 在开环控制中不必要，但启动它们不会影响运行。
        如需只启动 LiDAR + 串口桥，使用 --no-startup 并在外部手动启动。
        """
        self.get_logger().info("[启动] 启动前置节点 (LiDAR / ICP / Nav2 / 串口桥)...")
        start_prerequisites()

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

        # 记录检测到的 cube 位置（供放置参考）
        cube_pos = None

        # ── 启动 cube_detector（先启动，让它积累点云帧） ──
        self._start_cube_detector()
        self.get_logger().info("[抓取] 等待 cube_detector 初始化 (1.5s)...")
        time.sleep(1.5)

        # ── 推进 STM32 状态机到 PICK ──
        # catch.py 内部也会发送这些命令，但我们提前发一次确保状态正确
        self.send_auto_cmd(AUTO_CMD_START)
        time.sleep(0.2)
        self.send_auto_cmd(AUTO_CMD_ARRIVED_BOX, target=1)
        self.get_logger().info("[抓取] 等待 STM32 进入 PICK 状态 (0.6s)...")
        time.sleep(0.6)

        # ── 检测立方体（等待 /detected_cube 话题） ──
        self.get_logger().info(f"[抓取] 等待 cube_detector 检测结果（最多 {GRAB_TIMEOUT:.0f}s）...")
        deadline = time.monotonic() + GRAB_TIMEOUT
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._detected_cube is not None:
                cube_pos = self._detected_cube
                cx, cy, cz = cube_pos.pose.position.x, cube_pos.pose.position.y, cube_pos.pose.position.z
                self.get_logger().info(
                    f"[抓取] ✅ 检测到立方体: x={cx:.3f}  y={cy:.3f}  z={cz:.3f}"
                )
                break

        if cube_pos is None:
            self.get_logger().warn("[抓取] ⚠ 未检测到立方体，仍启动 catch.py 尝试抓取")

        # ── 启动 catch.py 执行抓取 ──
        # catch.py 内部流程：稳定检测 → 推进 STM32 状态机(PICK) → 发坐标 → 等待 pick_done
        self._start_catch()

        # ── 等待抓取完成 ──
        if self._catch_proc:
            self.get_logger().info(
                f"[抓取] 等待 catch.py (PID={self._catch_proc.pid}) 完成..."
            )
            try:
                self._catch_proc.wait(timeout=GRAB_TIMEOUT)
                self.get_logger().info("[抓取] ✅ catch.py 正常退出（抓取完成）")
            except subprocess.TimeoutExpired:
                self.get_logger().warn(f"[抓取] ⏰ catch.py 超时 ({GRAB_TIMEOUT:.0f}s)")
                try:
                    self._catch_proc.kill()
                    self._catch_proc.wait(timeout=3.0)
                except Exception:
                    pass

        # ── 清理 ──
        self._cleanup_subprocesses()
        self.get_logger().info("[抓取] ✅ 抓取阶段完成")

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
# 入口
# ============================================================

def main():
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

    # 通过全局变量透传 CLI 参数（Node 用模块常量读取）
    global FORWARD_SPEED, APPROACH_TIME, CARRY_TIME
    FORWARD_SPEED = args.vx
    if args.approach is not None:
        APPROACH_TIME = args.approach
    if args.carry is not None:
        CARRY_TIME = args.carry

    rclpy.init()
    node = OpenLoopAutoNode(no_startup=args.no_startup)

    try:
        node.step_open_gate()
        node.step_approach()

        # 记录起始阶段耗时，供调整参考
        t0 = time.monotonic()

        node.step_grab()

        t_grab = time.monotonic() - t0
        node.get_logger().info(f"[耗时] 抓取阶段用时 {t_grab:.1f}s")

        # 短暂停让机器人、机械臂稳定
        time.sleep(1.0)

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
