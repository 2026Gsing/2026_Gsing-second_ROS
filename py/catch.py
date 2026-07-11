#!/usr/bin/env python3
"""
catch.py — 机械臂抓取控制节点（通过串口桥转发）

功能：
  1. 订阅 /detected_cube (Marker) 获取立方体位置
  2. 将雷达坐标系坐标变换到机械臂坐标系（含偏移补偿）
  3. 使用滑动窗口标准差法判断位置是否稳定
  4. 位置稳定后，先推进 STM32 自动任务状态机到 PICK 状态
  5. 再通过 /vision/arm_control 发布坐标给串口桥转发给 STM32
  6. 以 2Hz 心跳重发确保下位机收到指令

自动任务状态机推进：
  catch.py 单独运行时，STM32 的 auto_task 默认在 IDLE 状态。
  Auto_Task_ArmAcceptsNewTarget() 要求只能是 PICK 或 PLACE 状态才接受 0x12 坐标，
  所以需要先发两条 auto_cmd 推进状态机：
    AUTO_CMD_START (cmd=1)      → IDLE → START → NAV_TO_BOX
    AUTO_CMD_ARRIVED_BOX (cmd=2) → NAV_TO_BOX → ARRIVED_BOX (400ms settle) → PICK
  进入 PICK 后，再发 0x12 坐标，机械臂才会执行抓取。

坐标变换：
  雷达系 (unilidar_lidar): x=前进, y=左, z=上
  机械臂系 (STM32 arm.c): x=高度(向上), y=侧向(向右), z=前向(向前)
  变换: final_x =  radar_z + offset_x
        final_y = -radar_y + offset_y
        final_z =  radar_x + offset_z

串口协议（由 cmd_vel_chassis_serial.py 转发）：
  [0x55][0xAA][0x12][len=12][x(float32)][y(float32)][z(float32)][checksum]

依赖：
  cube_detector.py（提供 /detected_cube 话题）
  cmd_vel_chassis_serial.py（串口桥，转发 /vision/arm_control → STM32）

使用方式：
  python3 py/catch.py
"""

import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from visualization_msgs.msg import Marker
from std_msgs.msg import String
import json
import math
import time
import numpy as np

# ==================== 坐标偏移参数 ====================
# 雷达→机械臂坐标系的物理安装偏移补偿（需标定）
# arm_x(高度) = -radar_z(高)     - OFFSET_X   # LiDAR 与臂肩的高度差
# arm_y(侧向) = -radar_y(左→右)   # LiDAR 与臂肩的左右偏移
# arm_z(前向) =  -radar_x(前)    - OFFSET_Z   # LiDAR 与臂肩的前后偏移
OFFSET_X = 0.0
OFFSET_Y = 0.0
OFFSET_Z = 0.25

# ==================== 箱子尺寸参数 ====================
BOX_HEIGHT = 0.25                # 箱子边长 250mm

HALF_BOX_HEIGHT = BOX_HEIGHT / 2  # 半高，中心→顶面补偿

# ==================== 工作空间限制 ====================
# 机械臂可达范围（与 STM32 Arm_IK 一致：hu=0.30, hl=0.32）
ARM_WORKSPACE_RADIUS_MAX = 0.62   # hu + hl
ARM_WORKSPACE_RADIUS_MIN = 0.02   # |hu - hl|

# ==================== 稳定检测参数 ====================
# 滑动窗口标准差法：位置跳动小于阈值时视为稳定
# 实际阈值 = STABLE_THRESHOLD_XY × STD_FACTOR
# 默认 0.08 × 1.0 = 0.08m = 8cm 标准差内视为稳定
STABLE_THRESHOLD_XY = 0.08    # XY 标准差阈值 (m)
STD_FACTOR = 1                 # 系数调节
STABLE_COUNT_REQUIRED = 3     # 需要连续多少帧稳定
CACHE_MAX_SIZE = 10            # 位置缓存最大帧数

# ==================== 自动任务事件常量 ====================
AUTO_CMD_START = 1
AUTO_CMD_ARRIVED_BOX = 2
AUTO_CMD_ARRIVED_ZONE = 4

# ARRIVED_BOX settle 400ms + 200ms 裕量，等 STM32 进入 PICK
ARM_DELAY_AFTER_ARRIVE_SEC = 0.6


# ==================== 状态机定义 ====================
class ArmState:
    """机械臂状态"""
    IDLE = "IDLE"               # 空闲，等待检测到立方体
    STARTING = "STARTING"       # 正在推进自动任务状态机
    WAITING_PICK = "WAITING_PICK"  # 等待 STM32 进入 PICK 状态
    COMPLETED = "COMPLETED"     # 已发送坐标，等待下次抓取


class ArmStateMachine(Node):
    """机械臂状态机：订阅立方体位置 → 稳定检测 → 推进状态机 → 通过串口桥转发"""

    def __init__(self):
        super().__init__('arm_state_machine')

        # catch.py only publishes ROS topics. The serial bridge owns the serial port.
        serial_device = os.environ.get("STM32_SERIAL_PORT", "/dev/ttyACM0")
        if not os.path.exists(serial_device):
            self.get_logger().warn(
                f"{serial_device} not found; catch.py will still publish /vision/arm_control. "
                "Start cmd_vel_chassis_serial.py with the real serial_port."
            )

        # ============ 发布器（通过串口桥转发） ============
        self.arm_pub = self.create_publisher(String, "/vision/arm_control", 10)
        self.auto_cmd_pub = self.create_publisher(String, "/vision/auto_cmd", 10)

        # ============ 状态机变量 ============
        self.state = ArmState.IDLE          # 初始状态：空闲
        self.target_position = None         # 稳定的目标位置

        # ============ 稳定检测缓存 ============
        self.position_cache = []            # 位置历史缓存（滑动窗口）

        # ============ 订阅立方体检测结果 ============
        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, depth=10)
        self.cube_sub = self.create_subscription(
            Marker, '/detected_cube', self.cube_callback, qos
        )

        # ============ 定时器 ============
        self.status_timer = self.create_timer(5.0, self.status_callback)  # 状态打印
        self.resend_timer = self.create_timer(0.5, self.resend_callback)  # 2Hz 心跳重发

        self.get_logger().info("=" * 60)
        self.get_logger().info("Arm State Machine Started (含自动任务状态机推进)")
        self.get_logger().info(f"State: {self.state}")
        self.get_logger().info("=" * 60)

    def send_auto_cmd(self, cmd, target=0, zone=0):
        """发布自动任务事件到 /vision/auto_cmd → 串口桥转发 0x15 → STM32"""
        msg = String()
        msg.data = json.dumps({"cmd": cmd, "target": target, "zone": zone})
        self.auto_cmd_pub.publish(msg)
        self.get_logger().info(f">>> 已发布 AUTO_CMD: cmd={cmd} target={target} zone={zone}")

    def transform_and_offset(self, radar_x, radar_y, radar_z):
        """
        坐标变换：雷达坐标系 → 机械臂坐标系（STM32 arm.c 正解定义）

        雷达系 (unilidar_lidar):
          x = 前进方向, y = 左侧, z = 上方

        机械臂系 (arm.c:382-384):
          x = 高度 (向上为正)
          y = 侧向 (向右为正)
          z = 前向 (向前为正)

        变换逻辑：
          arm_x (高度) = -radar_z (高度)     - OFFSET_X
          arm_y (侧向) = -radar_y (左→反转为右)
          arm_z (前向) = -radar_x (前进)     - OFFSET_Z
        """
        final_x = -radar_z - OFFSET_X      # radar_z(高度) → arm_x(高度)
        final_y = -radar_y                  # radar_y(左) → -arm_y(右)，无偏移
        final_z = -radar_x - OFFSET_Z       # radar_x(前进) → arm_z(前向)
        return final_x, final_y, final_z, (radar_x, radar_y, radar_z)

    def find_stable_points(self, positions, threshold_xy, required_count):
        """
        滑动窗口标准差法判断位置是否稳定

        原理：
          对最近的 required_count 个位置的 XY 坐标计算标准差。
          如果标准差 < 阈值，说明位置已收敛，返回均值作为最终目标。

        优势：
          O(n) 时间复杂度（比暴力组合搜索 O(C(n,k)) 快得多）
        """
        if len(positions) < required_count:
            return None, None

        recent = positions[-required_count:]
        arr = np.array(recent)

        std_x = np.std(arr[:, 0])  # X 方向跳动标准差
        std_y = np.std(arr[:, 1])  # Y 方向跳动标准差

        # 两个方向标准差都小于阈值 → 视为稳定
        if std_x < threshold_xy * STD_FACTOR and std_y < threshold_xy * STD_FACTOR:
            avg = (float(np.mean(arr[:, 0])),
                   float(np.mean(arr[:, 1])),
                   float(np.mean(arr[:, 2])))
            return list(range(len(recent))), avg

        return None, None

    def send_arm_position(self, x, y, z):
        """通过 /vision/arm_control 发布坐标 → 串口桥转发 → STM32"""
        msg = String()
        msg.data = json.dumps({"x": x, "y": y, "z": z})
        self.arm_pub.publish(msg)
        self.get_logger().info(f">>> 已发布坐标到 /vision/arm_control: ({x:.3f}, {y:.3f}, {z:.3f})")

    def cube_callback(self, msg):
        """
        立方体检测回调

        流程：
        1. 只有 IDLE 状态才处理新检测结果
        2. 坐标变换（雷达系 → 机械臂系）
        3. 加入缓存滑动窗口
        4. 调用 find_stable_points 检测是否稳定
        5. 稳定后，先发 AUTO_CMD 推进 STM32 状态机 → 延迟 → 发坐标
        """
        if self.state != ArmState.IDLE:
            return

        # 坐标变换
        radar_x, radar_y, radar_z = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
        final_x, final_y, final_z, _ = self.transform_and_offset(radar_x, radar_y, radar_z)
        final_x += HALF_BOX_HEIGHT  # 中心 → 顶面（在 arm 系 X 轴上加半高）

        # 工作空间检查
        dist = math.sqrt(final_x**2 + final_y**2 + final_z**2)
        if dist > ARM_WORKSPACE_RADIUS_MAX or dist < ARM_WORKSPACE_RADIUS_MIN:
            self.get_logger().error(f"坐标超出机械臂工作空间: "
                                    f"({final_x:.3f}, {final_y:.3f}, {final_z:.3f}) "
                                    f"dist={dist:.3f} (范围 {ARM_WORKSPACE_RADIUS_MIN}~{ARM_WORKSPACE_RADIUS_MAX})")
            return

        # 加入缓存
        self.position_cache.append((final_x, final_y, final_z))
        if len(self.position_cache) > CACHE_MAX_SIZE:
            self.position_cache.pop(0)

        self.get_logger().info(f"缓存 #{len(self.position_cache)}/{CACHE_MAX_SIZE}: "
                               f"原始({radar_x:.3f}, {radar_y:.3f}, {radar_z:.3f}) → "
                               f"最终({final_x:.3f}, {final_y:.3f}, {final_z:.3f})")

        # 稳定检测
        indices, avg_pos = self.find_stable_points(
            self.position_cache, STABLE_THRESHOLD_XY, STABLE_COUNT_REQUIRED)

        if indices is not None:
            self.get_logger().info("=" * 60)
            self.get_logger().info(f"✓ 视觉检测稳定！锁定平均目标: "
                                   f"({avg_pos[0]:.3f}, {avg_pos[1]:.3f}, {avg_pos[2]:.3f})")
            self.get_logger().info("=" * 60)

            self.target_position = avg_pos
            self.state = ArmState.STARTING

            # 第一步：推进自动任务状态机 → PICK
            self.get_logger().info("[SEQ] 发送 AUTO_CMD_START...")
            self.send_auto_cmd(AUTO_CMD_START)
            time.sleep(0.1)
            self.get_logger().info("[SEQ] 发送 AUTO_CMD_ARRIVED_BOX...")
            self.send_auto_cmd(AUTO_CMD_ARRIVED_BOX, target=1, zone=0)

            # 第二步：等 STM32 进入 PICK（ARRIVED_BOX settle 400ms + 裕量）
            self.state = ArmState.WAITING_PICK
            self.get_logger().info(f"[SEQ] 等待 {ARM_DELAY_AFTER_ARRIVE_SEC}s 让 STM32 进入 PICK 状态...")
            self.delayed_timer = self.create_timer(ARM_DELAY_AFTER_ARRIVE_SEC, self.send_delayed_arm_command)

    def send_delayed_arm_command(self):
        """定时器回调：等待 STM32 进入 PICK 后发送机械臂坐标（只执行一次）"""
        # 取消定时器，防止重复触发
        if hasattr(self, 'delayed_timer') and self.delayed_timer is not None:
            self.delayed_timer.cancel()
            self.delayed_timer = None

        if self.state != ArmState.WAITING_PICK or self.target_position is None:
            return

        self.get_logger().info("[SEQ] STM32 应已进入 PICK，发送机械臂坐标...")
        self.state = ArmState.COMPLETED
        self.send_arm_position(
            self.target_position[0],
            self.target_position[1],
            self.target_position[2]
        )

    def resend_callback(self):
        """2Hz 心跳重发：确保下位机收到坐标指令（防止串口丢包）"""
        if self.state == ArmState.COMPLETED and self.target_position is not None:
            self.get_logger().info("[Heartbeat] 保障心跳发送...")
            self.send_arm_position(
                self.target_position[0], self.target_position[1], self.target_position[2]
            )

    def status_callback(self):
        """5 秒定时器：打印当前状态"""
        self.get_logger().info(f"[STATUS] State: {self.state}")


def main(args=None):
    rclpy.init(args=args)
    node = ArmStateMachine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
