#!/usr/bin/env python3
"""
test_move.py — 纯轮足底盘运动测试工具

功能：
  1. 通过 /vision/auto_cmd 发送 AUTO_CMD_START 开启"门禁"
  2. 通过 /vision_cmd_vel 发送速度指令测试前进/后退/左转/右转/组合运动
  3. 可选的自动测试序列 + 手动单步模式

使用方式：
  前置条件：STM32 已上电站立、串口桥已启动
    ros2 run dog_nav2_bringup cmd_vel_chassis_serial --ros-args -p serial_port:=/dev/ttyACM0

cd /home/hyper/program/2026_Gsing-second_ROS/nav2_ws1
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch dog_nav2_bringup chassis_serial_bridge.launch.py \
  serial_port:=/dev/ttyACM0 \
  baud_rate:=115200 \
  cmd_vel_topic:=/cmd_vel \
  send_rate_hz:=50.0 \
  active_state:=1 \
  idle_state:=0

  运行本脚本：
    python3 py/test_move.py                          # 交互模式
    python3 py/test_move.py --auto                   # 自动测试序列
    python3 py/test_move.py --auto --duration 2.0    # 自定义每步持续时间

注意：
  - 确保底盘周围 2m 内无人员/障碍物
  - 首次运行建议先手动单步测试
  - 纯轮模式下腿部保持站立姿态，只有轮子转动
"""

import argparse
import json
import math
import sys
import threading
import time
from enum import IntEnum

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, UInt8

try:
    import serial
except ImportError:
    serial = None


# ============================================================
# AUTO_CMD 常量（与 STM32 protocol_handler.h 严格一致）
# ============================================================
class AutoCmd(IntEnum):
    NONE = 0
    START = 1
    ARRIVED_BOX = 2
    PICK_DONE = 3
    ARRIVED_ZONE = 4
    PLACE_DONE = 5
    NEXT = 6
    FINISH = 7
    ESTOP = 8

# ============================================================
# Gait IDs（0x11 步态切换，与 GaitMode_e 枚举一致）
# ============================================================
GAIT_TROT  = 0   # 小跑步态（四拍对角）
GAIT_WALK  = 1   # 行走步态
GAIT_BOUND = 2   # 跳跃步态
GAIT_PRONK = 3   # 弹跳步态
GAIT_NAMES = {0: "TROT(小跑)", 1: "WALK(行走)", 2: "BOUND(跳跃)", 3: "PRONK(弹跳)"}

# 注意：0x11 控制的是步态类型（腿怎么走），不是底盘模式。
# 底盘模式（纯轮/轮腿/纯足）由 STM32 内部 chassis_mode 控制，
# 在竞赛模式下由 apply_auto_chassis_profile() 固定为纯轮。


# ============================================================
# 测试序列定义
# ============================================================
# (名称, vx, wz, 描述)
TestStep = [
    ("STOP",       0.0,   0.0,  "停止"),
    ("FORWARD_SLOW",   0.40, 0.0,  "慢速前进 (vx=0.40)"),
    ("STOP",       0.0,   0.0,  "停止"),
    ("FORWARD_FAST",   0.80, 0.0,  "快速前进 (vx=0.80)"),
    ("STOP",       0.0,   0.0,  "停止"),
    ("BACKWARD_SLOW", -0.40, 0.0,  "慢速后退 (vx=-0.40)"),
    ("STOP",       0.0,   0.0,  "停止"),
    ("BACKWARD_FAST", -0.80, 0.0,  "快速后退 (vx=-0.80)"),
    ("STOP",       0.0,   0.0,  "停止"),
    ("ARC_LEFT_S",    0.40,  0.5,  "左弧慢 (vx=0.4, wz=+0.5)"),
    ("STOP",       0.0,   0.0,  "停止"),
    ("ARC_RIGHT_S",   0.40, -0.5,  "右弧慢 (vx=0.4, wz=-0.5)"),
    ("STOP",       0.0,   0.0,  "停止"),
    ("ARC_LEFT_M",    0.40,  1.0,  "左弧中 (vx=0.4, wz=+1.0)"),
    ("STOP",       0.0,   0.0,  "停止"),
    ("ARC_RIGHT_M",   0.40, -1.0,  "右弧中 (vx=0.4, wz=-1.0)"),
    ("STOP",       0.0,   0.0,  "停止"),
    ("ARC_LEFT_F",    0.40,  2.0,  "左弧快 (vx=0.4, wz=+2.0)"),
    ("STOP",       0.0,   0.0,  "停止"),
    ("ARC_RIGHT_F",   0.40, -2.0,  "右弧快 (vx=0.4, wz=-2.0)"),
    ("STOP",       0.0,   0.0,  "停止"),
]


class MoveTestNode(Node):
    def __init__(self, auto_mode=False, step_duration=1.5):
        super().__init__("test_move")

        # ======================== 发布器 ========================
        self.vel_pub = self.create_publisher(Twist, "/vision_cmd_vel", 10)
        self.auto_cmd_pub = self.create_publisher(String, "/vision/auto_cmd", 10)
        self.gait_pub = self.create_publisher(UInt8, "/vision/gait_cmd", 10)

        # ======================== 状态 ========================
        self.auto_mode = auto_mode
        self.step_duration = step_duration
        self.gate_open = False
        self._step_idx = 0

        # ======================== 持续重发机制 ========================
        # 目标速度（由 send_velocity 更新，_republish_cb 以 20Hz 持续重发）
        # 防止 STM32 串口桥的 500ms 视觉超时自动停车
        self._target_vx = 0.0
        self._target_wz = 0.0
        self._repub_timer = self.create_timer(0.05, self._republish_cb)  # 20Hz

        # 后台 spin 线程，确保 input() 阻塞时定时器仍能触发
        self._spin_thread = threading.Thread(target=self._bg_spin, daemon=True)
        self._spin_thread.start()

        self.get_logger().info("=" * 60)
        self.get_logger().info("  纯轮足底盘运动测试工具")
        self.get_logger().info("=" * 60)
        self.get_logger().info("")
        self.get_logger().info("【安全须知】")
        self.get_logger().info("  确保底盘周围 2m 内无人员/障碍物！")
        self.get_logger().info("  随时可按 Ctrl+C 急停")
        self.get_logger().info("")

        if auto_mode:
            self.get_logger().info("  自动测试模式")
            self.get_logger().info(f"  每步持续 {step_duration:.1f}s")
            self.get_logger().info("  按 Ctrl+C 中断测试")
        else:
            self.get_logger().info("  交互模式 — 输入命令:")
            self.get_logger().info("    1          → 开门 (AUTO_CMD_START)")
            self.get_logger().info("    2~11       → 速度预设 (带开门)")
            self.get_logger().info("    s          → 停止")
            self.get_logger().info("    直接输数字   → 前进到该速度")
        self.get_logger().info("")

    # ================================================================
    # 命令发送
    # ================================================================
    def _bg_spin(self):
        """后台 spin 线程，确保定时器在 input() 阻塞时仍能触发"""
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    def _republish_cb(self):
        """20Hz 定时器回调：持续重发当前目标速度，防止 500ms 视觉超时停车"""
        msg = Twist()
        msg.linear.x = self._target_vx
        msg.angular.z = self._target_wz
        self.vel_pub.publish(msg)

    def send_auto_cmd(self, cmd: int, target_id: int = 0, zone_id: int = 0):
        """发送 0x15 自动任务命令"""
        payload = json.dumps({
            "cmd": cmd & 0xFF,
            "target": target_id & 0xFF,
            "zone": zone_id & 0xFF,
        })
        msg = String()
        msg.data = payload
        self.auto_cmd_pub.publish(msg)
        self.get_logger().info(f"[AUTO] cmd={cmd} ({cmd:#x})")

    def send_velocity(self, vx: float, wz: float):
        """更新目标速度（定时器以 20Hz 持续重发，直到下次调用或停止）"""
        self._target_vx = float(vx)
        self._target_wz = float(wz)
        # 立即发一次，避免等待定时器下次触发
        msg = Twist()
        msg.linear.x = self._target_vx
        msg.angular.z = self._target_wz
        self.vel_pub.publish(msg)
        self.get_logger().info(f"[VEL] vx={vx:+.3f}  wz={wz:+.3f}")

    def open_gate(self):
        """发送 START 开启门禁 → 允许底盘运动"""
        if self.gate_open:
            return
        self.send_auto_cmd(AutoCmd.START)
        self.get_logger().info("  → 门禁已开，现在可以发送速度指令")
        self.gate_open = True
        time.sleep(0.2)

    def estop(self):
        """急停"""
        self.send_velocity(0.0, 0.0)
        self.send_auto_cmd(AutoCmd.ESTOP)
        self.get_logger().info("  ⛔ 急停！")
        self.gate_open = False

    def print_status(self):
        """打印当前状态信息"""
        self.get_logger().info("")
        self.get_logger().info("  ─── 状态 ───")
        self.get_logger().info(f"  门禁:      {'✅ 已开' if self.gate_open else '⛔ 未开'}")
        self.get_logger().info(f"  目标速度:  vx={self._target_vx:+.3f}  wz={self._target_wz:+.3f}")
        self.get_logger().info(f"  自动模式:  {'是' if self.auto_mode else '否'}")
        self.get_logger().info(f"  20Hz 重发: {'运行中' if hasattr(self, '_repub_timer') else '未启动'}")

    def send_gait(self, gait_id: int):
        """发送 0x11 步态切换命令（通过串口桥转发）"""
        name = GAIT_NAMES.get(gait_id & 0xFF, f"UNKNOWN(0x{gait_id:02x})")
        msg = UInt8()
        msg.data = gait_id & 0xFF
        self.gait_pub.publish(msg)
        self.get_logger().info(f"[GAIT] 发送 gait_id={gait_id} ({name})")

    def send_raw_serial_frame(self, port: str = "/dev/ttyACM0", baud: int = 115200,
                              vx: float = 0.0, wz: float = 0.0):
        """直接打开串口发送一帧 0x10 速度指令（不依赖串口桥）"""
        if serial is None:
            self.get_logger().error("  pyserial 未安装: sudo apt install python3-serial")
            return
        try:
            ser = serial.Serial(port=port, baudrate=baud, timeout=0.5)
            # 组帧
            import struct
            state = 1 if vx > 0 else 2 if vx < 0 else 3 if wz > 0 else 4 if wz < 0 else 0
            payload = struct.pack("<2fB", float(vx), float(wz), state)
            frame = bytes([0x55, 0xAA, 0x10, 0x09]) + payload
            checksum = sum(frame) & 0xFF
            frame += bytes([checksum])
            ser.write(frame)
            ser.flush()
            ser.close()
            self.get_logger().info(f"[RAW] 已直写串口 {port}: vx={vx:+.3f} wz={wz:+.3f}")
        except Exception as e:
            self.get_logger().error(f"[RAW] 串口直写失败: {e}")

    # ================================================================
    # 交互模式
    # ================================================================
    def _print_menu(self):
        print()
        print("  ╔═══════════════════════════════════════╗")
        print("  ║        纯轮足运动测试 — 菜单          ║")
        print("  ╠═══════════════════════════════════════╣")
        print("  ║  1 ║ 开门 (AUTO_CMD_START)            ║")
        print("  ║  2 ║ 前进 0.4                         ║")
        print("  ║  3 ║ 前进 0.8                         ║")
        print("  ║  4 ║ 后退 0.4                         ║")
        print("  ║  5 ║ 后退 0.8                         ║")
        print("  ║  6 ║ 左弧 0.5  (vx=0.4, +0.5)        ║")
        print("  ║  7 ║ 右弧 0.5  (vx=0.4, -0.5)        ║")
        print("  ║  8 ║ 左弧 1.0  (vx=0.4, +1.0)        ║")
        print("  ║  9 ║ 右弧 1.0  (vx=0.4, -1.0)        ║")
        print("  ║ 10 ║ 左弧 2.0  (vx=0.4, +2.0)        ║")
        print("  ║ 11 ║ 右弧 2.0  (vx=0.4, -2.0)        ║")
        print("  ║  s ║ 停止                             ║")
        print("  ╚═══════════════════════════════════════╝")
        print()

    def run_interactive(self):
        """CLI 交互模式 — 数字菜单"""
        self._print_menu()
        if not self.gate_open:
            self.get_logger().info("  ⚠ 尚未开门！请先选 1 开门")
        while rclpy.ok():
            try:
                raw = input("  > ").strip().lower()
            except (EOFError, KeyboardInterrupt):
                print()
                break

            if raw == "q":
                break

            # --- 字母命令 ---
            if raw == "1" or raw == "g":
                self.open_gate()
            elif raw == "." or raw == "s":
                self.send_velocity(0.0, 0.0)

            # --- 数字命令 ---
            elif raw == "2":
                self.open_gate()
                self.send_velocity(0.40, 0.0)
            elif raw == "3":
                self.open_gate()
                self.send_velocity(0.80, 0.0)
            elif raw == "4":
                self.open_gate()
                self.send_velocity(-0.40, 0.0)
            elif raw == "5":
                self.open_gate()
                self.send_velocity(-0.80, 0.0)
            elif raw == "6":
                self.open_gate()
                self.send_velocity(0.40, 0.5)
            elif raw == "7":
                self.open_gate()
                self.send_velocity(0.40, -0.5)
            elif raw == "8":
                self.open_gate()
                self.send_velocity(0.40, 1.0)
            elif raw == "9":
                self.open_gate()
                self.send_velocity(0.40, -1.0)
            elif raw == "10":
                self.open_gate()
                self.send_velocity(0.40, 2.0)
            elif raw == "11":
                self.open_gate()
                self.send_velocity(0.40, -2.0)

            # --- 自定义速度: 直接输数字 ---
            else:
                try:
                    v = float(raw)
                    if v <= 0:
                        self.send_velocity(0.0, 0.0)
                    else:
                        self.open_gate()
                        self.send_velocity(v, 0.0)
                except ValueError:
                    print(f"  未知命令: {raw}，输入 1-11, s")

        self.send_velocity(0.0, 0.0)

    # ================================================================
    # 自动测试序列
    # ================================================================
    def run_auto_sequence(self):
        """执行预定义的自动测试序列"""
        # 先开门
        self.open_gate()
        time.sleep(0.5)

        for i, (name, vx, wz, desc) in enumerate(TestStep):
            if not rclpy.ok():
                break

            self.get_logger().info(f"")
            self.get_logger().info(f"[{i+1}/{len(TestStep)}] {desc}")
            self.send_velocity(vx, wz)

            # 等待指定时长，期间 spinning
            deadline = time.monotonic() + self.step_duration
            while time.monotonic() < deadline and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.05)

        # 结束：停止
        self.send_velocity(0.0, 0.0)
        self.get_logger().info(f"")
        self.get_logger().info("=" * 60)
        self.get_logger().info("  自动测试序列完成！")
        self.get_logger().info("=" * 60)

    # ================================================================
    # 快速单步测试（用于外部程序调用）
    # ================================================================
    def step(self, vx: float, wz: float, duration: float = 1.0):
        """执行单步运动：发速度 → 等待 → 停止"""
        self.open_gate()
        self.send_velocity(vx, wz)
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
        self.send_velocity(0.0, 0.0)
        time.sleep(0.3)


# ================================================================
# 入口
# ================================================================
def main(args=None):
    parser = argparse.ArgumentParser(description="纯轮足底盘运动测试工具")
    parser.add_argument("--auto", action="store_true", help="自动测试序列模式")
    parser.add_argument("--duration", type=float, default=1.5, help="自动测试每步持续时间 (秒)")
    parser.add_argument("--vx", type=float, help="快速单步: 指定 vx")
    parser.add_argument("--wz", type=float, help="快速单步: 指定 wz")
    parser.add_argument("--time", type=float, default=2.0, help="快速单步: 持续时间")
    parsed, _ = parser.parse_known_args()

    rclpy.init()
    node = MoveTestNode(auto_mode=parsed.auto, step_duration=parsed.duration)

    try:
        if parsed.vx is not None or parsed.wz is not None:
            # 命令行单步模式
            vx = parsed.vx or 0.0
            wz = parsed.wz or 0.0
            node.step(vx, wz, parsed.time)
        elif parsed.auto:
            # 自动测试序列
            node.run_auto_sequence()
        else:
            # 交互模式
            node.run_interactive()
    except KeyboardInterrupt:
        node.get_logger().info("\n  用户中断")
    finally:
        node.send_velocity(0.0, 0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
