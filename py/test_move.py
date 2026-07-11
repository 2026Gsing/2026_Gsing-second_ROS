#!/usr/bin/env python3
"""
test_move.py — 纯轮足底盘运动测试工具

功能：
  1. 自动启动串口桥（chassis_serial_bridge）
  2. 通过 /vision/auto_cmd 发送 AUTO_CMD_START 开启"门禁"
  3. 通过 /vision_cmd_vel 发送速度指令测试前进/后退/左转/右转/组合运动
  4. 可选的自动测试序列 + 手动单步模式
1
使用方式：
  STM32 已上电站立，串口桥由脚本自动启动。

  运行本脚本：
    python3 py/test_move.py                          # 交互模式
    python3 py/test_move.py --auto                   # 自动测试序列
    python3 py/test_move.py --auto --duration 2.0    # 自定义每步持续时
    python3 py/test_move.py --no-serial              # 跳过自动启动串口桥

注意：
  - 确保底盘周围 2m 内无人员/障碍物
  - 首次运行建议先手动单步测试
  - 纯轮模式下腿部保持站立姿态，只有轮子转动
"""

import argparse
import json
import math
import os
import subprocess
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
# 机器人状态枚举（与 STM32 control.h RobotState_e 严格一致）
# ============================================================
ROBOT_STATE_IDLE     = 0  # 空闲/停止
ROBOT_STATE_FORWARD  = 1  # 前进
ROBOT_STATE_BACKWARD = 2  # 后退
ROBOT_STATE_LEFT     = 3  # 左转
ROBOT_STATE_RIGHT    = 4  # 右转

_STATE_EPSILON = 1e-6


def derive_robot_state(vx: float, wz: float) -> int:
    """
    从 vx, wz 速度矢量推导机器人运动状态。
    逻辑与 cmd_vel_chassis_serial.py 的 derive_robot_state() 一致：
    优先判断角速度（自转），再判断线速度（平移），否则返回 IDLE。
    """
    if abs(wz) > abs(vx) and abs(wz) > _STATE_EPSILON:
        return ROBOT_STATE_LEFT if wz >= 0.0 else ROBOT_STATE_RIGHT
    if abs(vx) > _STATE_EPSILON:
        return ROBOT_STATE_FORWARD if vx >= 0.0 else ROBOT_STATE_BACKWARD
    return ROBOT_STATE_IDLE


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
    def __init__(self, auto_mode=False, step_duration=1.5, no_serial=False):
        super().__init__("test_move")

        # ======================== 发布器 ========================
        self.vel_pub = self.create_publisher(Twist, "/vision_cmd_vel", 10)
        self.auto_cmd_pub = self.create_publisher(String, "/vision/auto_cmd", 10)
        self.gait_pub = self.create_publisher(UInt8, "/vision/gait_cmd", 10)
        self.state_pub = self.create_publisher(UInt8, "/vision/robot_state", 10)

        # ======================== 状态 ========================
        self.auto_mode = auto_mode
        self.step_duration = step_duration
        self.gate_open = False
        self._step_idx = 0
        self._serial_proc = None

        # ======================== 50Hz 持续重发机制 ========================
        # 目标速度（由 send_velocity 更新，_republish_cb 以 50Hz 持续重发）
        # 防止 STM32 串口桥的 500ms 视觉超时自动停车
        self._target_vx = 0.0
        self._target_wz = 0.0
        self._target_state = ROBOT_STATE_IDLE
        self._repub_timer = self.create_timer(0.02, self._republish_cb)  # 50Hz

        # 后台 spin 线程，确保 input() 阻塞时定时器仍能触发
        self._spin_thread = threading.Thread(target=self._bg_spin, daemon=True)
        self._spin_thread.start()

        # ======================== 自动启动串口桥 ========================
        if not no_serial:
            self._launch_serial_bridge()
        else:
            self.get_logger().info("  ⏭ 跳过串口桥自动启动 (--no-serial)")

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
            self.get_logger().info("    1~4  → 速度预设 (自动开门)")
            self.get_logger().info("    s    → 停止")
            self.get_logger().info("    直接输 vx wz → 自定义速度 (自动开门)")
        self.get_logger().info("")

    # ================================================================
    # 自动启动串口桥
    # ================================================================
    def _try_serial_port(self, port: str) -> subprocess.Popen | None:
        """尝试在指定串口上启动串口桥，成功返回进程，失败返回 None"""
        nav2_ws = "/home/hyper/program/2026_Gsing-second_ROS/nav2_ws1"
        setup_script = os.path.join(nav2_ws, "install/setup.bash")
        launch_file = os.path.join(
            nav2_ws, "src/dog_nav2_bringup",
            "launch/chassis_serial_bridge.launch.py"
        )

        cmd = (
            f"bash -c '"
            f"source /opt/ros/jazzy/setup.bash && "
            f"source {setup_script} && "
            f"export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && "
            f"ros2 launch {launch_file} "
            f"serial_port:={port} "
            f"baud_rate:=115200 "
            f"cmd_vel_topic:=/cmd_vel "
            f"send_rate_hz:=50.0 "
            f"active_state:=1 "
            f"idle_state:=0"
            f"'"
        )

        self.get_logger().info(f"  → 尝试 {port} ...")
        proc = subprocess.Popen(
            cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            cwd=nav2_ws
        )
        time.sleep(1.0)
        if proc.poll() is not None:
            self.get_logger().warn(f"  ⚠ {port} 不可用")
            proc.kill()
            return None
        return proc

    def _launch_serial_bridge(self):
        """依次尝试 ACM0 → ACM1，找到可用串口即启动串口桥"""
        nav2_ws = "/home/hyper/program/2026_Gsing-second_ROS/nav2_ws1"
        launch_file = os.path.join(
            nav2_ws, "src/dog_nav2_bringup",
            "launch/chassis_serial_bridge.launch.py"
        )
        if not os.path.exists(launch_file):
            self.get_logger().warn(f"  ⚠ 串口桥启动文件不存在: {launch_file}")
            return

        for port in ("/dev/ttyACM0", "/dev/ttyACM1"):
            if self._serial_proc is not None:
                break
            self._serial_proc = self._try_serial_port(port)

        if self._serial_proc is not None:
            self.get_logger().info(f"  ✅ 串口桥已启动")
        else:
            self.get_logger().error("  ❌ ACM0 和 ACM1 均不可用，请检查串口连接")

    def _bg_spin(self):
        """后台 spin 线程，确保定时器在 input() 阻塞时仍能触发"""
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    def _republish_cb(self):
        """20Hz 定时器回调：持续重发当前目标速度 + 状态，防止 500ms 视觉超时停车"""
        msg = Twist()
        msg.linear.x = self._target_vx
        msg.angular.z = self._target_wz
        self.vel_pub.publish(msg)
        state_msg = UInt8()
        state_msg.data = self._target_state
        self.state_pub.publish(state_msg)

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
        self._target_state = derive_robot_state(vx, wz)
        # 立即发一次，避免等待定时器下次触发
        msg = Twist()
        msg.linear.x = self._target_vx
        msg.angular.z = self._target_wz
        self.vel_pub.publish(msg)
        state_msg = UInt8()
        state_msg.data = self._target_state
        self.state_pub.publish(state_msg)
        state_names = {0: "IDLE", 1: "FORWARD", 2: "BACKWARD", 3: "LEFT", 4: "RIGHT"}
        self.get_logger().info(
            f"[VEL] vx={vx:+.3f}  wz={wz:+.3f}  state={self._target_state}"
            f"({state_names.get(self._target_state, '?')})"
        )

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
        state_names = {0: "IDLE", 1: "FORWARD", 2: "BACKWARD", 3: "LEFT", 4: "RIGHT"}
        self.get_logger().info("")
        self.get_logger().info("  ─── 状态 ───")
        self.get_logger().info(f"  门禁:      {'✅ 已开' if self.gate_open else '⛔ 未开'}")
        self.get_logger().info(f"  目标速度:  vx={self._target_vx:+.3f}  wz={self._target_wz:+.3f}")
        self.get_logger().info(f"  状态:      {self._target_state} ({state_names.get(self._target_state, '?')})")
        self.get_logger().info(f"  自动模式:  {'是' if self.auto_mode else '否'}")
        self.get_logger().info(f"  20Hz 重发: {'运行中' if hasattr(self, '_repub_timer') else '未启动'}")

    def send_gait(self, gait_id: int):
        """发送 0x11 步态切换命令（通过串口桥转发）"""
        name = GAIT_NAMES.get(gait_id & 0xFF, f"UNKNOWN(0x{gait_id:02x})")
        msg = UInt8()
        msg.data = gait_id & 0xFF
        self.gait_pub.publish(msg)
        self.get_logger().info(f"[GAIT] 发送 gait_id={gait_id} ({name})")

    def send_arm_mission(self, mode: int, flags: int,
                          pick=None, back=None, place=None):
        """
        发送机械臂多段任务（通过串口桥转发为 0x14 FUNC_ARM_MISSION）

        参数:
          mode: 任务模式
          flags: 位标志 0x01=HAS_PICK, 0x02=HAS_BACK, 0x04=HAS_PLACE
          pick: (x, y, z) 抓取坐标 or None
          back: (x, y, z) 取回坐标 or None
          place: (x, y, z) 放置坐标 or None
        """
        data = {"mode": mode & 0xFF, "flags": flags & 0xFF}
        for key, val in [("pick", pick), ("back", back), ("place", place)]:
            if val is not None:
                data[key] = [float(v) for v in val]
        msg = String()
        msg.data = json.dumps(data)
        arm_pub = self.create_publisher(String, "/vision/arm_mission", 10)
        arm_pub.publish(msg)
        self.destroy_publisher(arm_pub)
        self.get_logger().info(
            f"[ARM_MISSION] mode={mode} flags={flags} "
            f"{'pick='+str(pick) if pick else ''} "
            f"{'back='+str(back) if back else ''} "
            f"{'place='+str(place) if place else ''}"
        )

    def send_raw_serial_frame(self, port: str = "/dev/ttyACM0", baud: int = 115200,
                              vx: float = 0.0, wz: float = 0.0):
        """直接打开串口发送一帧 0x10 速度指令（不依赖串口桥）"""
        if serial is None:
            self.get_logger().error("  pyserial 未安装: sudo apt install python3-serial")
            return
        try:
            ser = serial.Serial(port=port, baudrate=baud, timeout=0.5)
            # 组帧（使用与串口桥一致的 derive_robot_state 逻辑）
            import struct
            state = derive_robot_state(vx, wz)
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
        print("  ║        运动测试 — 菜单                ║")
        print("  ╠═══════════════════════════════════════╣")
        print("  ║  ── 运动 ──                           ║")
        print("  ║  1 ║ 前进 0.4                         ║")
        print("  ║  2 ║ 后退 0.4                         ║")
        print("  ║  3 ║ 左弧 1.0                         ║")
        print("  ║  4 ║ 右弧 1.0                         ║")
        print("  ║  s ║ 停止                             ║")
        print("  ║  直接输 vx wz → 自定义速度            ║")
        print("  ╚═══════════════════════════════════════╝")
        print()

    def run_interactive(self):
        """CLI 交互模式 — 数字菜单"""
        self._print_menu()
        while rclpy.ok():
            try:
                raw = input("  > ").strip().lower()
            except (EOFError, KeyboardInterrupt):
                print()
                break

            if raw == "q":
                break

            # --- 字母命令 ---
            if raw == "s":
                self.send_velocity(0.0, 0.0)

            # --- 数字命令 ---
            elif raw == "1":
                self.open_gate()
                self.send_velocity(0.40, 0.0)
            elif raw == "2":
                self.open_gate()
                self.send_velocity(-0.40, 0.0)
            elif raw == "3":
                self.open_gate()
                self.send_velocity(0.40, 1.0)
            elif raw == "4":
                self.open_gate()
                self.send_velocity(0.40, -1.0)

            # --- 自定义速度: 输 vx 或 vx wz (空格分隔) ---
            else:
                try:
                    parts = raw.split()
                    vx = float(parts[0])
                    wz = float(parts[1]) if len(parts) > 1 else 0.0
                    if vx == 0 and wz == 0:
                        self.send_velocity(0.0, 0.0)
                    else:
                        self.open_gate()
                        self.send_velocity(vx, wz)
                except (ValueError, IndexError):
                    print(f"  格式: vx 或 vx wz  (如: 0.4 或 0.4 0.5)")

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
    parser.add_argument("--vx", type=float, help="指定前进速度")
    parser.add_argument("--speed", type=float, help="快捷指定前进速度（等同 --vx）")
    parser.add_argument("--wz", type=float, help="指定转向速度")
    parser.add_argument("--no-serial", action="store_true", help="不自动启动串口桥")
    parser.add_argument("--time", type=float, default=0, help="持续时间（秒，0=持续运行直到 Ctrl+C）")
    parsed, _ = parser.parse_known_args()

    # --speed 是 --vx 的快捷写法
    if parsed.speed is not None and parsed.vx is None:
        parsed.vx = parsed.speed

    rclpy.init()
    node = MoveTestNode(auto_mode=parsed.auto, step_duration=parsed.duration, no_serial=parsed.no_serial)

    try:
        if parsed.vx is not None or parsed.wz is not None:
            # 命令行模式
            vx = parsed.vx or 0.0
            wz = parsed.wz or 0.0
            node.open_gate()
            node.send_velocity(vx, wz)
            if parsed.time > 0:
                # 指定时长后停止
                deadline = time.monotonic() + parsed.time
                while time.monotonic() < deadline and rclpy.ok():
                    rclpy.spin_once(node, timeout_sec=0.05)
                node.send_velocity(0.0, 0.0)
            else:
                # 持续运行直到 Ctrl+C（后台线程已负责 spinning）
                node.get_logger().info(f"  持续运行中，按 Ctrl+C 停止")
                while rclpy.ok():
                    time.sleep(0.1)
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
        # 关闭串口桥子进程
        if node._serial_proc is not None:
            node.get_logger().info("  关闭串口桥...")
            node._serial_proc.terminate()
            try:
                node._serial_proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                node._serial_proc.kill()
                node._serial_proc.wait()
            node.get_logger().info("  ✅ 串口桥已关闭")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
