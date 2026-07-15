#!/usr/bin/env python3
"""
auto_task.py — 简化版自动任务

流程（不依赖 Nav2 导航）：
  1. 启动前置节点（LiDAR/ICP/串口桥）
  2. loop 2 次：
     a. 前移到箱子（开环 vx, 可调）
     b. 抓取（等待 arm_event pick_done）
     c. 前移到归位区（开环 vx, 可调）
     d. 放置（等待 arm_event place_done）
     e. 后退到下一个箱子（开环 -vx, 可调）

运行时调参：
  ros2 param set /auto_task_simple approach_time 8.0
  ros2 param set /auto_task_simple zone_time 25.0
  ros2 param set /auto_task_simple back_time 15.0
  ros2 param set /auto_task_simple move_speed 0.2
"""

import json
import os
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from std_msgs.msg import String

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from launch_utils import start_prerequisites, cleanup_all

# ============ 串口协议常量 ============
AUTO_CMD_START = 1
AUTO_CMD_ARRIVED_BOX = 2
AUTO_CMD_ARRIVED_ZONE = 4
AUTO_CMD_FINISH = 7

ARM_EVENT_PICK_DONE = 1
ARM_EVENT_PLACE_DONE = 2

ARM_MISSION_PICK_TO_BACK = 1
ARM_MISSION_BACK_TO_PLACE = 2
ARM_MISSION_HAS_PICK = 0x01
ARM_MISSION_HAS_BACK = 0x02
ARM_MISSION_HAS_PLACE = 0x04

NUM_BOXES = 2

# 开环控制参数（改这里）
MOVE_SPEED = 0.2        # m/s
APPROACH_TIME = 5.0     # s: 起点→箱子
PICK_TIMEOUT = 30.0     # s: 抓取超时
ZONE_TIME = 18.0        # s: 箱子→归位区
PLACE_TIMEOUT = 25.0    # s: 放置超时


class AutoTaskSimple(Node):
    def __init__(self):
        super().__init__("auto_task_simple")

        # 发布器
        self.pub_cmd_vel = self.create_publisher(Twist, "/vision_cmd_vel", 10)
        self.pub_auto_cmd = self.create_publisher(String, "/vision/auto_cmd", 10)
        self.pub_arm_mission = self.create_publisher(String, "/vision/arm_mission", 10)
        self.pub_initialpose = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )

        # 订阅器
        self.arm_event = None
        self.arm_event_time = 0.0
        self.create_subscription(String, "/vision/arm_event", self._arm_event_cb, 10)

        self.get_logger().info("auto_task_simple 已启动")

    # ── 事件回调 ──────────────────────────────────────────────
    def _arm_event_cb(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        self.arm_event = data
        self.arm_event_time = time.monotonic()
        name = data.get("event_name", "")
        if name == "pick_done":
            self.get_logger().info(
                f"[ARM_EVENT] pick_done: slot={data.get('back_slot')} "
                f"side={data.get('back_side_name')}"
            )
        elif name == "place_done":
            self.get_logger().info("[ARM_EVENT] place_done")

    # ── 串口指令发送 ──────────────────────────────────────────
    def send_vel(self, vx, wz):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(wz)
        self.pub_cmd_vel.publish(msg)

    def send_auto_cmd(self, cmd, target=0, zone=0):
        data = json.dumps({"cmd": cmd, "target": target, "zone": zone})
        self.pub_auto_cmd.publish(String(data=data))
        self.get_logger().info(f"  AUTO_CMD: cmd={cmd} target={target} zone={zone}")

    def send_arm_mission(self, mode, flags, sequences):
        payload = {"mode": mode, "flags": flags}
        for key in ("pick", "back", "place"):
            if key in sequences:
                payload[key] = sequences[key]
        self.pub_arm_mission.publish(String(data=json.dumps(payload)))
        self.get_logger().info(f"  ARM_MISSION: mode={mode} flags={flags}")

    def publish_initialpose(self, x=0.0, y=0.0):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.w = 1.0
        self.pub_initialpose.publish(msg)

    # ── 开环控制 ──────────────────────────────────────────────
    def drive(self, speed, duration):
        vx = MOVE_SPEED * (1 if speed >= 0 else -1)
        self.get_logger().info(f"  行驶 vx={vx:.2f} × {duration:.0f}s = {vx*duration:.1f}m")
        self.send_vel(vx, 0.0)
        time.sleep(duration)
        self.send_vel(0.0, 0.0)
        self.get_logger().info("  停止")

    # ── 等待 ARM_EVENT ────────────────────────────────────────
    def wait_arm_event(self, expected_name, timeout):
        self.arm_event = None
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.arm_event:
                if self.arm_event.get("event_name", "") == expected_name:
                    return True
        self.get_logger().warn(f"  ⚠ 等待 {expected_name} 超时 ({timeout:.0f}s)")
        return False

    # ── 等待串口桥就绪 ────────────────────────────────────────
    def wait_serial_ready(self, timeout=10.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.pub_auto_cmd.get_subscription_count() > 0:
                self.get_logger().info("  串口桥已就绪")
                return True
            time.sleep(0.2)
        self.get_logger().warn("  ⚠ 串口桥未检测到（仍继续）")
        return False


def main():
    rclpy.init()
    print("╔══════════════════════════════════════════════╗")
    print("║  自动任务（简化版）                           ║")
    print("╚══════════════════════════════════════════════╝")

    start_prerequisites()

    node = AutoTaskSimple()

    print("\n[启动] 等待 8s 让 ICP/串口桥就绪...")
    time.sleep(8)

    node.publish_initialpose(0, 0)
    node.get_logger().info("[启动] /initialpose 已发布 (0,0,0)")
    time.sleep(1)

    node.wait_serial_ready()

    # ══════════════════════════════════════════════════════
    # 主循环：抓取 NUM_BOXES 个箱子
    # ══════════════════════════════════════════════════════
    node.send_auto_cmd(AUTO_CMD_START)
    time.sleep(0.5)

    # ══════════════════════════════════════════════════════
    # 阶段 1: 前进到箱子区域，连续抓取 2 个
    # ══════════════════════════════════════════════════════
    print(f"\n[阶段 1] 前进到箱子区域 ({APPROACH_TIME:.0f}s)...")
    node.drive(1, APPROACH_TIME)

    for i in range(NUM_BOXES):
        side = "left" if i == 0 else "right"
        back_side = 1 if i == 0 else 2
        print(f"\n{'═'*40}")
        print(f"  抓取 {i+1}/{NUM_BOXES} → {side}侧")
        print(f"{'═'*40}")

        node.send_auto_cmd(AUTO_CMD_ARRIVED_BOX, target=i + 1)
        print("  等待 STM32 进入 PICK 状态...")
        time.sleep(1.0)

        print(f"  抓取（超时 {PICK_TIMEOUT:.0f}s）...")
        node.send_arm_mission(
            mode=ARM_MISSION_PICK_TO_BACK,
            flags=ARM_MISSION_HAS_PICK | ARM_MISSION_HAS_BACK | 0x08,
            sequences={
                "pick": {"x": -0.21, "y": 0.25, "z": -0.4},
                "back": {"x": -0.25, "y": 0.0, "z": -0.35},
                "back_side": back_side,
            },
        )
        if node.wait_arm_event("pick_done", PICK_TIMEOUT):
            print("  ✅ 抓取成功")
        else:
            print("  ⚠ 抓取超时，继续")
        time.sleep(0.5)

    # ══════════════════════════════════════════════════════
    # 阶段 2: 前进到归位区，连续放置 2 个
    # ══════════════════════════════════════════════════════
    print(f"\n[阶段 2] 前进到归位区 ({ZONE_TIME:.0f}s)...")
    node.drive(1, ZONE_TIME)

    for i in range(NUM_BOXES):
        zone_id = 0 if i == 0 else 1
        side = "left" if i == 0 else "right"
        print(f"\n{'═'*40}")
        print(f"  放置 {i+1}/{NUM_BOXES} → {side}侧 → 归位区 {zone_id}")
        print(f"{'═'*40}")

        node.send_auto_cmd(AUTO_CMD_ARRIVED_ZONE, target=i + 1, zone=zone_id)
        print("  等待 STM32 进入 PLACE 状态...")
        time.sleep(1.0)

        print(f"  放置（超时 {PLACE_TIMEOUT:.0f}s）...")
        node.send_arm_mission(
            mode=ARM_MISSION_BACK_TO_PLACE,
            flags=ARM_MISSION_HAS_PLACE,
            sequences={
                "place": {"x": -0.25, "y": 0.0, "z": -0.35},
            },
        )
        if node.wait_arm_event("place_done", PLACE_TIMEOUT):
            print("  ✅ 放置成功")
        else:
            print("  ⚠ 放置超时，继续")
        time.sleep(0.5)

    # 完成
    print(f"\n{'═'*50}")
    print("  自动任务完成 ✅")
    print(f"{'═'*50}")
    node.send_auto_cmd(AUTO_CMD_FINISH)
    time.sleep(0.5)

    node.destroy_node()
    rclpy.shutdown()
    cleanup_all()


if __name__ == "__main__":
    main()
