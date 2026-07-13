#!/usr/bin/env python3
"""
catch.py — 机械臂抓取控制节点（通过串口桥转发）

功能：
  1. 订阅 /detected_cube (Marker) 获取立方体位置
  2. 将雷达坐标系坐标变换到机械臂坐标系（含偏移补偿）
  3. 使用滑动窗口标准差法判断位置是否稳定
  4. 位置稳定后，先推进 STM32 自动任务状态机到 PICK 状态
  5. 再通过 /vision/arm_control 发布坐标给串口桥转发给 STM32

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
        final_y =  radar_y + offset_y
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
# arm_y(侧向) = radar_y
# arm_z(前向) =  -radar_x(前)    - OFFSET_Z   # LiDAR 与臂肩的前后偏移
OFFSET_X = 0.110
OFFSET_Y = 0.0
OFFSET_Z = 0.25

# ==================== 箱子尺寸参数 ====================
BOX_HEIGHT = 0.25                # 箱子边长 250mm
HALF_BOX_HEIGHT = BOX_HEIGHT / 2  # 半高，中心→顶面补偿

# ==================== 工作空间限制 ====================
# 与 STM32 Arm_IK 一致：hu=0.30, hl=0.32 → 可达范围 [0.02, 0.62]
# STM32 内部还会对 X 加 0.03 (suction_cup_l) 后检查 ARM_TARGET_X/Y/Z_MIN/MAX：
#   X∈[-0.20, 0.45], Y∈[-0.50, 0.50], Z∈[-0.75, 0.55]
# 这里预扣 0.03 以匹配 STM32 的最终检查结果
ARM_TARGET_X_MIN = -0.23      # -0.20 - 0.03
ARM_TARGET_X_MAX =  0.42      #  0.45 - 0.03
ARM_TARGET_Y_MIN = -0.50
ARM_TARGET_Y_MAX =  0.50
ARM_TARGET_Z_MIN = -0.75
ARM_TARGET_Z_MAX =  0.55
ARM_DIST_MAX = 0.62
ARM_DIST_MIN = 0.02

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
ARM_EVENT_PICK_DONE = 1

# ARRIVED_BOX settle 400ms + 200ms 裕量，等 STM32 进入 PICK
ARM_DELAY_AFTER_ARRIVE_SEC = 0.6
ARM_EVENT_WAIT_TIMEOUT_SEC = 20.0


# ============================================================
# 模块级工具函数（可被 vision_auto_task_node.py 直接导入使用）
# ============================================================

def transform_and_offset(radar_x, radar_y, radar_z):
    """
    坐标变换：雷达坐标系 → 机械臂坐标系

    雷达系 (unilidar_lidar): x=前进, y=左, z=上
    机械臂系 (arm.c):       x=高度(向上), y=侧向(向右), z=前向(向前)
    """
    final_x = -radar_z - OFFSET_X
    final_y = radar_y
    final_z = -radar_x - OFFSET_Z
    return final_x, final_y, final_z, (radar_x, radar_y, radar_z)


def find_stable_points(positions, threshold_xy, required_count):
    """
    滑动窗口标准差法判断位置是否稳定。

    Args:
        positions: list of (x, y, z) tuples
        threshold_xy: XY 标准差阈值
        required_count: 需要多少帧稳定

    Returns:
        (indices, avg_pos) 或 (None, None)
    """
    if len(positions) < required_count:
        return None, None

    recent = positions[-required_count:]
    arr = np.array(recent)

    std_x = np.std(arr[:, 0])
    std_y = np.std(arr[:, 1])

    if std_x < threshold_xy * STD_FACTOR and std_y < threshold_xy * STD_FACTOR:
        avg = (float(np.mean(arr[:, 0])),
               float(np.mean(arr[:, 1])),
               float(np.mean(arr[:, 2])))
        return list(range(len(recent))), avg

    return None, None


def validate_arm_target(x, y, z):
    """
    检查坐标是否在 STM32 机械臂工作空间内。
    匹配 second-DM4340 的 arm_target_in_range() + Arm_IK() 约束。
    """
    if not all(math.isfinite(v) for v in (x, y, z)):
        return False, f"无效值: ({x}, {y}, {z})"

    if not (ARM_TARGET_X_MIN <= x <= ARM_TARGET_X_MAX):
        return False, f"X={x:.3f}m 超限 [{ARM_TARGET_X_MIN:.2f}, {ARM_TARGET_X_MAX:.2f}]"
    if not (ARM_TARGET_Y_MIN <= y <= ARM_TARGET_Y_MAX):
        return False, f"Y={y:.3f}m 超限 [{ARM_TARGET_Y_MIN:.2f}, {ARM_TARGET_Y_MAX:.2f}]"
    if not (ARM_TARGET_Z_MIN <= z <= ARM_TARGET_Z_MAX):
        return False, f"Z={z:.3f}m 超限 [{ARM_TARGET_Z_MIN:.2f}, {ARM_TARGET_Z_MAX:.2f}]"

    dist = math.sqrt(x * x + y * y + z * z)
    if not (ARM_DIST_MIN <= dist <= ARM_DIST_MAX):
        return False, f"距离={dist:.3f}m 超限 [{ARM_DIST_MIN:.2f}, {ARM_DIST_MAX:.2f}]"

    # Arm_IK 肩下禁区：Z>=0 时要求 X>=0（前伸不能低于肩关节）
    if z >= 0 and x < 0:
        return False, f"肩下禁区: Z={z:.3f}>=0 时 X={x:.3f}<0，IK 无解"

    return True, "OK"


def stm32_will_accept(x, y, z):
    """
    模拟 STM32 接收 0x12 目标后的拒绝逻辑。
    STM32 内部：add suction_cup_l(0.03) to X → arm_target_in_range → Arm_IK
    """
    comp_x = x + 0.03
    comp_y = y
    comp_z = z

    if not (-0.20 <= comp_x <= 0.45):
        return False, f"STM32 侧 X={comp_x:.3f} 超 [-0.20, 0.45]（补偿后）"
    if not (-0.50 <= comp_y <= 0.50):
        return False, f"Y={comp_y:.3f} 超 [-0.50, 0.50]"
    if not (-0.75 <= comp_z <= 0.55):
        return False, f"Z={comp_z:.3f} 超 [-0.75, 0.55]"

    dist = math.sqrt(comp_x * comp_x + comp_y * comp_y + comp_z * comp_z)
    if not (0.02 <= dist <= 0.62):
        return False, f"STM32 侧距离={dist:.3f} 超 [0.02, 0.62]（补偿后）"

    return True, "OK"


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
        self.command_sent_time = 0.0
        self._shutdown_requested = False

        # ============ 稳定检测缓存 ============
        self.position_cache = []            # 位置历史缓存（滑动窗口）

        # ============ 订阅立方体检测结果 ============
        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, depth=10)
        self.cube_sub = self.create_subscription(
            Marker, '/detected_cube', self.cube_callback, qos
        )
        self.arm_event_sub = self.create_subscription(
            String, '/vision/arm_event', self.arm_event_callback, 10
        )

        # ============ 定时器 ============
        self.status_timer = self.create_timer(5.0, self.status_callback)  # 状态打印

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
        """委托模块级函数"""
        return transform_and_offset(radar_x, radar_y, radar_z)

    def find_stable_points(self, positions, threshold_xy, required_count):
        """委托模块级函数"""
        return find_stable_points(positions, threshold_xy, required_count)

    def send_arm_position(self, x, y, z):
        """通过 /vision/arm_control 发布坐标 → 串口桥转发 → STM32"""
        msg = String()
        msg.data = json.dumps({"x": x, "y": y, "z": z})
        self.arm_pub.publish(msg)
        self.get_logger().info(f">>> 已发布坐标到 /vision/arm_control: ({x:.3f}, {y:.3f}, {z:.3f})")

    def arm_event_callback(self, msg):
        """接收 STM32 完成抓取后的 0x22 回传事件。"""
        try:
            data = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f"[ARM_EVENT] JSON 解析失败: {e}")
            return

        event = int(data.get("event", 0))
        event_name = data.get("event_name", "")
        if event != ARM_EVENT_PICK_DONE and event_name != "pick_done":
            return

        side_name = data.get("back_side_name", "unknown")
        back_slot = int(data.get("back_slot", 0))
        self.get_logger().info(
            f"[ARM_EVENT] 抓取完成: back_slot={back_slot} side={side_name}"
        )
        if not self._shutdown_requested:
            self._shutdown_requested = True
            rclpy.shutdown()

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

        # 检查坐标合法性（匹配 STM32 工作空间限制）
        valid, reason = validate_arm_target(final_x, final_y, final_z)
        if not valid:
            self.get_logger().error(
                f"⚠️ 目标超限（{reason}），清除缓存重新采集\n"
                f"  坐标: ({final_x:.3f}, {final_y:.3f}, {final_z:.3f}) "
                f"原始: ({radar_x:.3f}, {radar_y:.3f}, {radar_z:.3f})"
            )
            self.position_cache.clear()
            return

        # STM32 接收预判（吸盘补偿+0.03后检测）
        stm32_ok, stm32_reason = stm32_will_accept(final_x, final_y, final_z)
        if not stm32_ok:
            self.get_logger().error(
                f"⚠️ STM32 将拒绝此目标（{stm32_reason}），清除缓存重新采集\n"
                f"  坐标: ({final_x:.3f}, {final_y:.3f}, {final_z:.3f})"
            )
            self.position_cache.clear()
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
        self.command_sent_time = time.monotonic()
        self.get_logger().info("[等待] 坐标已发送，等待 STM32 回传 ARM_EVENT pick_done")

    def status_callback(self):
        """5 秒定时器：打印当前状态"""
        self.get_logger().info(f"[STATUS] State: {self.state}")
        if (
            self.state == ArmState.COMPLETED
            and self.command_sent_time > 0.0
            and not self._shutdown_requested
            and (time.monotonic() - self.command_sent_time) > ARM_EVENT_WAIT_TIMEOUT_SEC
        ):
            self.get_logger().warn("[ARM_EVENT] 等待 pick_done 超时，退出 catch.py")
            self._shutdown_requested = True
            rclpy.shutdown()


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
