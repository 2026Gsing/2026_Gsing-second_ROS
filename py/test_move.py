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
import time
from enum import IntEnum

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


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
# 测试序列定义
# ============================================================
# (名称, vx, wz, 描述)
TestStep = [
    ("STOP",       0.0,   0.0,  "停止"),
    ("FORWARD_SLOW",   0.15, 0.0,  "慢速前进 (vx=0.15)"),
    ("STOP",       0.0,   0.0,  "停止"),
    ("FORWARD_MED",    0.30, 0.0,  "中速前进 (vx=0.30)"),
    ("STOP",       0.0,   0.0,  "停止"),
    ("FORWARD_FAST",   0.50, 0.0,  "快速前进 (vx=0.50)"),
    ("STOP",       0.0,   0.0,  "停止"),
    ("BACKWARD_SLOW", -0.15, 0.0,  "慢速后退 (vx=-0.15)"),
    ("STOP",       0.0,   0.0,  "停止"),
    ("BACKWARD_MED",  -0.30, 0.0,  "中速后退 (vx=-0.30)"),
    ("STOP",       0.0,   0.0,  "停止"),
    ("TURN_LEFT",     0.0,   0.5,  "原地左转 (wz=+0.5)"),
    ("STOP",       0.0,   0.0,  "停止"),
    ("TURN_RIGHT",    0.0,  -0.5,  "原地右转 (wz=-0.5)"),
    ("STOP",       0.0,   0.0,  "停止"),
    ("ARC_LEFT",      0.20,  0.3,  "左弧前进 (vx=0.2, wz=+0.3)"),
    ("STOP",       0.0,   0.0,  "停止"),
    ("ARC_RIGHT",     0.20, -0.3,  "右弧前进 (vx=0.2, wz=-0.3)"),
    ("STOP",       0.0,   0.0,  "停止"),
]


class MoveTestNode(Node):
    def __init__(self, auto_mode=False, step_duration=1.5):
        super().__init__("test_move")

        # ======================== 发布器 ========================
        self.vel_pub = self.create_publisher(Twist, "/vision_cmd_vel", 10)
        self.auto_cmd_pub = self.create_publisher(String, "/vision/auto_cmd", 10)

        # ======================== 状态 ========================
        self.auto_mode = auto_mode
        self.step_duration = step_duration
        self.gate_open = False
        self._step_idx = 0

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
            self.get_logger().info("    g          → 开门 (AUTO_CMD_START)")
            self.get_logger().info("    e          → 急停 (ESTOP)")
            self.get_logger().info("    f/r        → 前进/后退 0.3")
            self.get_logger().info("    l/r        → 左转/右转 0.5")
            self.get_logger().info("    a/d        → 左弧/右弧 0.2+0.3")
            self.get_logger().info("    s          → 停止")
            self.get_logger().info("    0~9        → 自定义速度 vx=0.X")
            self.get_logger().info("    auto       → 执行自动测试序列")
            self.get_logger().info("    q          → 退出")
        self.get_logger().info("")

    # ================================================================
    # 命令发送
    # ================================================================
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
        """发送速度指令 (0x10 底盘控制)"""
        msg = Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(wz)
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

    # ================================================================
    # 交互模式
    # ================================================================
    def _print_menu(self):
        print()
        print("  ╔═══════════════════════════════════════╗")
        print("  ║        纯轮足运动测试 — 菜单          ║")
        print("  ╠═══════════════════════════════════════╣")
        print("  ║  0 ║ 前进 vx=X (输入数字设速度)      ║")
        print("  ║  1 ║ 开门 (AUTO_CMD_START)            ║")
        print("  ║  2 ║ 前进 0.2                         ║")
        print("  ║  3 ║ 前进 0.4                         ║")
        print("  ║  4 ║ 后退 0.2                         ║")
        print("  ║  5 ║ 后退 0.4                         ║")
        print("  ║  6 ║ 原地左转 0.5                     ║")
        print("  ║  7 ║ 原地右转 0.5                     ║")
        print("  ║  8 ║ 左弧前进 (vx=0.2, wz=+0.3)      ║")
        print("  ║  9 ║ 右弧前进 (vx=0.2, wz=-0.3)      ║")
        print("  ║  . ║ 停止                             ║")
        print("  ║  e ║ 急停                             ║")
        print("  ║  a ║ 自动测试序列                     ║")
        print("  ║  q ║ 退出                             ║")
        print("  ╚═══════════════════════════════════════╝")
        print("  📌 直接输入 0.05~0.99 → 前进到该速度")
        print("  📌 例: 0.3 前进0.3，30 前进0.3")
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
            elif raw == "e":
                self.estop()
            elif raw == "." or raw == "s":
                self.send_velocity(0.0, 0.0)
            elif raw == "a":
                self.run_auto_sequence()

            # --- 数字命令 ---
            elif raw == "2":
                self.open_gate()
                self.send_velocity(0.20, 0.0)
            elif raw == "3":
                self.open_gate()
                self.send_velocity(0.40, 0.0)
            elif raw == "4":
                self.open_gate()
                self.send_velocity(-0.20, 0.0)
            elif raw == "5":
                self.open_gate()
                self.send_velocity(-0.40, 0.0)
            elif raw == "6":
                self.open_gate()
                self.send_velocity(0.0, 0.5)
            elif raw == "7":
                self.open_gate()
                self.send_velocity(0.0, -0.5)
            elif raw == "8":
                self.open_gate()
                self.send_velocity(0.20, 0.3)
            elif raw == "9":
                self.open_gate()
                self.send_velocity(0.20, -0.3)

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
                    print(f"  未知命令: {raw}，输入 0-9 或 . e a q")

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
