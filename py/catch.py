#!/usr/bin/env python3
"""
机械臂控制节点 - 滑动窗口标准差版
使用 np.std 替代暴力组合搜索，O(C(n,k)) → O(n)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from visualization_msgs.msg import Marker
import serial
import struct
import time
import numpy as np

# ==================== 串口配置 ====================
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200

# ==================== 坐标偏移参数 ====================
OFFSET_X = 0.06
OFFSET_Y = 0.0
OFFSET_Z = -0.15

# ==================== 稳定检测参数 ====================
# 标准差异常阈值 = STABLE_THRESHOLD_XY × STD_FACTOR
# 当前: 0.10 × 0.4 = 0.04 → 位置跳动 < 4cm 视为稳定
# 改大(如 0.10×1.0=0.10) → 更容易触发发送,但位置可能不准
# 改小(如 0.10×0.2=0.02) → 更难触发发送,但位置更准
STABLE_THRESHOLD_XY = 0.08    # ← 改这里可直接改变量
STD_FACTOR = 1              # ← 改这里可调系数
STABLE_COUNT_REQUIRED = 3
CACHE_MAX_SIZE = 10

# ==================== 状态机定义 ====================
class ArmState:
    IDLE = "IDLE"
    COMPLETED = "COMPLETED"

# ==================== 协议帧定义 ====================
HEAD1 = 0x55
HEAD2 = 0xAA
FUNC_ARM_CONTROL = 0x12

def build_frame(func_id, payload):
    frame = bytes([HEAD1, HEAD2, func_id, len(payload)]) + payload
    checksum = sum(frame) & 0xFF
    return frame + bytes([checksum])

def float_to_le(value):
    return struct.pack('<f', value)

def cmd_arm_control(x, y, z):
    payload = float_to_le(x) + float_to_le(y) + float_to_le(z)
    return build_frame(FUNC_ARM_CONTROL, payload)

class ArmStateMachine(Node):
    def __init__(self):
        super().__init__('arm_state_machine')

        # 串口配置
        self.serial = None
        self.serial_connected = False
        self.connect_serial()

        # 状态机变量
        self.state = ArmState.IDLE
        self.target_position = None

        # 稳定检测缓存
        self.position_cache = []

        # 订阅立方体检测
        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, depth=10)
        self.cube_sub = self.create_subscription(Marker, '/detected_cube', self.cube_callback, qos)

        # 定时器
        self.status_timer = self.create_timer(5.0, self.status_callback)
        self.resend_timer = self.create_timer(1.0, self.resend_callback)

        self.get_logger().info("=" * 60)
        self.get_logger().info("Arm State Machine Started (滑动窗口标准差版)")
        self.get_logger().info(f"State: {self.state}")
        self.get_logger().info("=" * 60)

    def transform_and_offset(self, radar_x, radar_y, radar_z):
        temp_x = -radar_z
        temp_y = radar_y
        temp_z = -radar_x
        final_x = temp_x + OFFSET_X
        final_y = temp_y + OFFSET_Y
        final_z = temp_z + OFFSET_Z
        return final_x, final_y, final_z, (temp_x, temp_y, temp_z)

    def find_stable_points(self, positions, threshold_xy, required_count):
        """
        滑动窗口标准差法（O(n) 非 O(C(n,k))）
        
        对缓存中最近 required_count 个位置的 XY 计算标准差，
        若标准差 < threshold_xy * 0.4，视为稳定并返回均值。
        """
        if len(positions) < required_count:
            return None, None

        recent = positions[-required_count:]
        arr = np.array(recent)

        std_x = np.std(arr[:, 0])
        std_y = np.std(arr[:, 1])

        # 标准差判据：等效于 pairwise distance < threshold_xy
        # 实际阈值 = 0.10 × 0.4 = 0.04m = 4cm
        if std_x < threshold_xy * STD_FACTOR and std_y < threshold_xy * STD_FACTOR:
            avg = (float(np.mean(arr[:, 0])),
                   float(np.mean(arr[:, 1])),
                   float(np.mean(arr[:, 2])))
            return list(range(len(recent))), avg

        return None, None

    def connect_serial(self):
        try:
            self.serial = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
            self.serial.dtr = False
            self.serial.rts = False
            time.sleep(0.5)
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
            self.serial_connected = True
            self.get_logger().info("✅ 串口连接并底层清理成功！")
            return True
        except Exception as e:
            self.serial_connected = False
            self.get_logger().error(f"❌ 串口连接失败: {e}")
            return False

    def send_arm_position(self, x, y, z):
        if not self.serial_connected:
            if not self.connect_serial():
                return False
        try:
            self.serial.write(cmd_arm_control(x, y, z))
            self.get_logger().info(f">>> 已向单片机发送坐标: ({x:.3f}, {y:.3f}, {z:.3f})")
            return True
        except Exception as e:
            self.get_logger().error(f"串口发送异常: {e}")
            self.serial_connected = False
            return False

    def cube_callback(self, msg):
        if self.state != ArmState.IDLE:
            return

        radar_x, radar_y, radar_z = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
        final_x, final_y, final_z, _ = self.transform_and_offset(radar_x, radar_y, radar_z)

        self.position_cache.append((final_x, final_y, final_z))

        if len(self.position_cache) > CACHE_MAX_SIZE:
            self.position_cache.pop(0)

        self.get_logger().info(f"缓存 #{len(self.position_cache)}/{CACHE_MAX_SIZE}: "
                               f"原始({radar_x:.3f}, {radar_y:.3f}, {radar_z:.3f}) → "
                               f"最终({final_x:.3f}, {final_y:.3f}, {final_z:.3f})")

        indices, avg_pos = self.find_stable_points(
            self.position_cache, STABLE_THRESHOLD_XY, STABLE_COUNT_REQUIRED)

        if indices is not None:
            self.get_logger().info("=" * 60)
            self.get_logger().info(f"✓ 视觉检测稳定！锁定平均目标: "
                                   f"({avg_pos[0]:.3f}, {avg_pos[1]:.3f}, {avg_pos[2]:.3f})")
            self.get_logger().info("=" * 60)

            self.target_position = avg_pos
            self.state = ArmState.COMPLETED
            self.send_arm_position(avg_pos[0], avg_pos[1], avg_pos[2])

    def resend_callback(self):
        if self.state == ArmState.COMPLETED and self.target_position is not None:
            self.get_logger().info("[Heartbeat] 保障心跳发送...")
            self.send_arm_position(self.target_position[0], self.target_position[1], self.target_position[2])

    def status_callback(self):
        self.get_logger().info(f"[STATUS] State: {self.state}")

    def cleanup(self):
        if self.serial and self.serial.is_open:
            self.serial.close()

def main(args=None):
    rclpy.init(args=args)
    node = ArmStateMachine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
