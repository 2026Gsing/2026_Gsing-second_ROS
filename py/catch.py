#!/usr/bin/env python3
"""
catch.py — 机械臂抓取控制节点（串口通信）

功能：
  1. 订阅 /detected_cube (Marker) 获取立方体位置
  2. 将雷达坐标系坐标变换到机械臂坐标系（含偏移补偿）
  3. 使用滑动窗口标准差法判断位置是否稳定
  4. 位置稳定后，通过串口发送坐标给 STM32 控制机械臂抓取
  5. 以 1Hz 心跳发送确保下位机收到指令

坐标变换：
  雷达系 (unilidar_lidar): x=前进, y=左, z=上
  机械臂系: x=右, y=前, z=下
  变换: final_x = -radar_z + offset_x
        final_y =  radar_y + offset_y
        final_z = -radar_x + offset_z

串口协议（机械臂控制）:
  [0x55][0xAA][0x12][len=12][x(float32)][y(float32)][z(float32)][checksum]

依赖：
  cube_detector.py（提供 /detected_cube 话题）
  STM32 串口（/dev/ttyACM0, 115200）

使用方式：
  ros2 run py catch.py
  或者：
  python3 py/catch.py
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
# 雷达→机械臂坐标系变换后的偏移补偿（标定值）
OFFSET_X = 0.06
OFFSET_Y = 0.0
OFFSET_Z = -0.15

# ==================== 稳定检测参数 ====================
# 滑动窗口标准差法：位置跳动小于阈值时视为稳定
# 实际阈值 = STABLE_THRESHOLD_XY × STD_FACTOR
# 默认 0.08 × 1.0 = 0.08m = 8cm 标准差内视为稳定
STABLE_THRESHOLD_XY = 0.08    # XY 标准差阈值 (m)
STD_FACTOR = 1                 # 系数调节
STABLE_COUNT_REQUIRED = 3      # 需要连续多少帧稳定
CACHE_MAX_SIZE = 10            # 位置缓存最大帧数

# ==================== 状态机定义 ====================
class ArmState:
    """机械臂状态"""
    IDLE = "IDLE"           # 空闲，等待检测到立方体
    COMPLETED = "COMPLETED" # 已发送坐标，等待下次抓取

# ==================== 协议帧定义 ====================
HEAD1 = 0x55
HEAD2 = 0xAA
FUNC_ARM_CONTROL = 0x12     # 功能码：机械臂控制

def build_frame(func_id, payload):
    """组装通用协议帧：[0x55][0xAA][func_id][len][payload][checksum]"""
    frame = bytes([HEAD1, HEAD2, func_id, len(payload)]) + payload
    checksum = sum(frame) & 0xFF
    return frame + bytes([checksum])

def float_to_le(value):
    """float32 → 小端字节序"""
    return struct.pack('<f', value)

def cmd_arm_control(x, y, z):
    """组装机械臂控制帧：3 个 float32 = 12 字节载荷"""
    payload = float_to_le(x) + float_to_le(y) + float_to_le(z)
    return build_frame(FUNC_ARM_CONTROL, payload)

class ArmStateMachine(Node):
    """机械臂状态机：订阅立方体位置 → 稳定检测 → 串口发送"""

    def __init__(self):
        super().__init__('arm_state_machine')

        # ============ 串口配置 ============
        self.serial = None
        self.serial_connected = False
        self.connect_serial()

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
        self.resend_timer = self.create_timer(1.0, self.resend_callback)  # 心跳重发

        self.get_logger().info("=" * 60)
        self.get_logger().info("Arm State Machine Started (滑动窗口标准差版)")
        self.get_logger().info(f"State: {self.state}")
        self.get_logger().info("=" * 60)

    def transform_and_offset(self, radar_x, radar_y, radar_z):
        """
        坐标变换：雷达坐标系 → 机械臂坐标系

        雷达系 (unilidar_lidar):
          x = 前进方向, y = 左侧, z = 上方

        机械臂系:
          x = 右侧, y = 前进方向, z = 下方

        变换公式：
          final_x = -radar_z + OFFSET_X    (雷达z反转→机械臂x)
          final_y =  radar_y + OFFSET_Y    (雷达y不变→机械臂y)
          final_z = -radar_x + OFFSET_Z    (雷达x反转→机械臂z)
        """
        temp_x = -radar_z
        temp_y = radar_y
        temp_z = -radar_x
        final_x = temp_x + OFFSET_X
        final_y = temp_y + OFFSET_Y
        final_z = temp_z + OFFSET_Z
        return final_x, final_y, final_z, (temp_x, temp_y, temp_z)

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

    def connect_serial(self):
        """连接 STM32 串口，清理缓冲区"""
        try:
            self.serial = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
            self.serial.dtr = False
            self.serial.rts = False
            time.sleep(0.5)  # 等待串口稳定
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
        """通过串口向 STM32 发送机械臂目标位置"""
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
        """
        立方体检测回调

        流程：
        1. 只有 IDLE 状态才处理新检测结果
        2. 坐标变换（雷达系 → 机械臂系）
        3. 加入缓存滑动窗口
        4. 调用 find_stable_points 检测是否稳定
        5. 稳定后状态 → COMPLETED，串口发送坐标
        """
        if self.state != ArmState.IDLE:
            return

        # 坐标变换
        radar_x, radar_y, radar_z = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
        final_x, final_y, final_z, _ = self.transform_and_offset(radar_x, radar_y, radar_z)

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
            self.state = ArmState.COMPLETED
            self.send_arm_position(avg_pos[0], avg_pos[1], avg_pos[2])

    def resend_callback(self):
        """1Hz 心跳重发：确保下位机收到坐标指令（防止串口丢包）"""
        if self.state == ArmState.COMPLETED and self.target_position is not None:
            self.get_logger().info("[Heartbeat] 保障心跳发送...")
            self.send_arm_position(
                self.target_position[0], self.target_position[1], self.target_position[2]
            )

    def status_callback(self):
        """5 秒定时器：打印当前状态"""
        self.get_logger().info(f"[STATUS] State: {self.state}")

    def cleanup(self):
        """清理：关闭串口"""
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
