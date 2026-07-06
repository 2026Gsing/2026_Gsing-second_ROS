#!/usr/bin/env python3
"""
cmd_vel_chassis_serial.py — Nav2 /cmd_vel + Vision → STM32 串口协议转发

功能：
  1. 订阅 /cmd_vel (Nav2) → 发送 0x10 底盘速度帧（原有功能）
  2. 订阅 /vision_cmd_vel → 发送 0x10（视觉精细对位，优先级高于 Nav2）
  3. 订阅 /vision/auto_cmd → 发送 0x15 自动任务事件帧（新增）
  4. 超时保护：底盘命令超时自动停车

协议帧格式 0x10（底盘速度）：
  [0x55][0xAA][0x10][0x09][vx(f32)][wz(f32)][state(u8)][CheckSum]

协议帧格式 0x15（自动任务事件）：
  [0x55][0xAA][0x15][0x03][cmd(u8)][target(u8)][zone(u8)][CheckSum]

安全机制：
  1. Nav2 速度超时 80ms → 自动停止
  2. 视觉速度超时 500ms → 退回 Nav2 控制
  3. 退出保护: 节点销毁时发送停止帧
"""

import json
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import String, UInt8

try:
    import serial
except ImportError as e:
    serial = None
    _SERIAL_IMPORT_ERROR = e
else:
    _SERIAL_IMPORT_ERROR = None

# ============ 串口协议常量 ============
HEAD1 = 0x55           # 帧头 1
HEAD2 = 0xAA           # 帧头 2
CMD_CHASSIS_VEL = 0x10 # 功能码：底盘速度控制
CMD_GAIT_SWITCH = 0x11 # 功能码：步态切换
CMD_AUTO_TASK = 0x15   # 功能码：自动任务事件
CMD_CHASSIS_MODE = 0x14 # 功能码：底盘模式切换（新增）
LEN_VEL_PAYLOAD = 9    # 载荷长度：vx(4) + wz(4) + state(1) = 9
LEN_AUTO_PAYLOAD = 3   # 载荷长度：cmd(1) + target(1) + zone(1) = 3
PACKET_FMT = "<2fB"    # 打包格式（小端）：float32(vx), float32(wz), uint8(state)

# 机器人状态枚举（与 STM32 control.h RobotState_e 严格一致）
ROBOT_STATE_IDLE     = 0  # 空闲/停止
ROBOT_STATE_FORWARD  = 1  # 前进
ROBOT_STATE_BACKWARD = 2  # 后退
ROBOT_STATE_LEFT     = 3  # 左转
ROBOT_STATE_RIGHT    = 4  # 右转

_STATE_EPSILON = 1e-6


def derive_robot_state(vx: float, wz: float) -> int:
    """
    从 vx, wz 速度矢量推导机器人运动状态。

    推导逻辑与 STM32 control.c derive_robot_state() 一致：
    优先判断角速度（自转），再判断线速度（平移），否则返回 IDLE。
    """
    # 角速度占主导 → 左转/右转
    if abs(wz) > abs(vx) and abs(wz) > _STATE_EPSILON:
        return ROBOT_STATE_LEFT if wz >= 0.0 else ROBOT_STATE_RIGHT
    # 线速度主导 → 前进/后退
    if abs(vx) > _STATE_EPSILON:
        return ROBOT_STATE_FORWARD if vx >= 0.0 else ROBOT_STATE_BACKWARD
    # 零速度 → 空闲
    return ROBOT_STATE_IDLE


class CmdVelChassisSerial(Node):
    def __init__(self):
        super().__init__("cmd_vel_chassis_serial")

        if serial is None:
            raise RuntimeError(
                "需要安装 pyserial: sudo apt install python3-serial"
            ) from _SERIAL_IMPORT_ERROR

        # ============ 参数声明 ============
        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("vision_cmd_vel_topic", "/vision_cmd_vel")
        self.declare_parameter("auto_cmd_topic", "/vision/auto_cmd")
        self.declare_parameter("gait_cmd_topic", "/vision/gait_cmd")
        self.declare_parameter("chassis_mode_topic", "/vision/chassis_mode")
        self.declare_parameter("send_rate_hz", 50.0)
        self.declare_parameter("stale_timeout_sec", 0.08)
        self.declare_parameter("vision_timeout_sec", 0.5)   # 视觉控速超时
        self.declare_parameter("zero_on_shutdown", True)
        self.declare_parameter("active_state", 1)
        self.declare_parameter("idle_state", 0)

        # 读取参数
        port = self.get_parameter("serial_port").get_parameter_value().string_value
        baud = self.get_parameter("baud_rate").get_parameter_value().integer_value
        topic = self.get_parameter("cmd_vel_topic").get_parameter_value().string_value
        vision_topic = self.get_parameter("vision_cmd_vel_topic").get_parameter_value().string_value
        auto_topic = self.get_parameter("auto_cmd_topic").get_parameter_value().string_value
        gait_topic = self.get_parameter("gait_cmd_topic").get_parameter_value().string_value
        mode_topic = self.get_parameter("chassis_mode_topic").get_parameter_value().string_value
        self._send_rate = max(
            10.0, self.get_parameter("send_rate_hz").get_parameter_value().double_value
        )
        self._stale_timeout = self.get_parameter("stale_timeout_sec").get_parameter_value().double_value
        self._vision_timeout = self.get_parameter("vision_timeout_sec").get_parameter_value().double_value
        self._zero_on_shutdown = (
            self.get_parameter("zero_on_shutdown").get_parameter_value().bool_value
        )

        # ============ 状态变量 ============
        self._lock = threading.Lock()
        self._last_twist = Twist()
        self._last_time = 0.0
        self._vision_twist = None       # 视觉控速（可选，优先于 Nav2）
        self._vision_last_time = 0.0

        # ============ 打开串口 ============
        self._ser = serial.Serial(port=port, baudrate=baud, timeout=0.05)
        self.get_logger().info(
            f"Opened {port} @ {baud}, cmd_vel={topic}, send={self._send_rate}Hz"
        )

        # ============ 订阅 ============
        # 使用显式 QoS 确保与 Nav2 controller 的发布 QoS 兼容
        _qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(Twist, topic, self._twist_cb, _qos)
        self.create_subscription(Twist, vision_topic, self._vision_cb, _qos)
        self.create_subscription(String, auto_topic, self._auto_cmd_cb, _qos)
        self.create_subscription(UInt8, gait_topic, self._gait_cb, _qos)
        self.create_subscription(UInt8, mode_topic, self._chassis_mode_cb, _qos)

        # ============ 定时发送 ============
        period = 1.0 / self._send_rate
        self._timer = self.create_timer(period, self._send_tick)

    def _twist_cb(self, msg: Twist):
        """接收 Nav2 的 /cmd_vel 消息，缓存最新值"""
        with self._lock:
            self._last_twist = msg
            self._last_time = time.monotonic()
            self.get_logger().info(
                f"收到 /cmd_vel: vx={msg.linear.x:.3f} wz={msg.angular.z:.3f}"
            )

    def _vision_cb(self, msg: Twist):
        """接收视觉 /vision_cmd_vel 消息（精细对位，覆盖 Nav2）"""
        with self._lock:
            self._vision_twist = msg
            self._vision_last_time = time.monotonic()

    def _auto_cmd_cb(self, msg: String):
        """接收 /vision/auto_cmd JSON → 组装 0x15 帧发送"""
        try:
            data = json.loads(msg.data)
            cmd = int(data.get("cmd", 0)) & 0xFF
            target = int(data.get("target", 0)) & 0xFF
            zone = int(data.get("zone", 0)) & 0xFF
            pkt = self._build_auto_packet(cmd, target, zone)
            with self._lock:
                self._ser.write(pkt)
                self._ser.flush()
            self.get_logger().info(
                f"[AUTO] 发送 0x15: cmd={cmd} target={target} zone={zone}"
            )
        except Exception as e:
            self.get_logger().error(f"[AUTO] 解析/发送失败: {e}")

    def _gait_cb(self, msg: UInt8):
        """接收 /vision/gait_cmd → 组装 0x11 帧发送"""
        try:
            gait_id = msg.data & 0xFF
            pkt = self._build_gait_packet(gait_id)
            with self._lock:
                self._ser.write(pkt)
                self._ser.flush()
            self.get_logger().info(f"[GAIT] 发送 0x11: gait_id={gait_id}")
        except Exception as e:
            self.get_logger().error(f"[GAIT] 发送失败: {e}")

    def _chassis_mode_cb(self, msg: UInt8):
        """接收 /vision/chassis_mode → 组装 0x14 帧发送"""
        try:
            mode_id = msg.data & 0xFF
            pkt = self._build_chassis_mode_packet(mode_id)
            with self._lock:
                self._ser.write(pkt)
                self._ser.flush()
            mode_names = {0: "纯轮", 1: "轮足", 2: "纯足"}
            name = mode_names.get(mode_id, f"UNKNOWN({mode_id})")
            self.get_logger().info(f"[MODE] 发送 0x14: mode={mode_id} ({name})")
        except Exception as e:
            self.get_logger().error(f"[MODE] 发送失败: {e}")

    def _build_packet(self, vx: float, wz: float, state: int) -> bytes:
        """
        组装 0x10 串口协议帧
        格式：[0x55][0xAA][0x10][0x09][vx(4B)][wz(4B)][state(1B)][checksum(1B)]
        """
        payload = struct.pack(PACKET_FMT, float(vx), float(wz), int(state) & 0xFF)
        frame_wo_checksum = bytes([HEAD1, HEAD2, CMD_CHASSIS_VEL, LEN_VEL_PAYLOAD]) + payload
        checksum = sum(frame_wo_checksum) & 0xFF
        return frame_wo_checksum + bytes([checksum])

    def _build_auto_packet(self, cmd: int, target: int, zone: int) -> bytes:
        """
        组装 0x15 串口协议帧
        格式：[0x55][0xAA][0x15][0x03][cmd(1B)][target(1B)][zone(1B)][checksum(1B)]
        """
        payload = bytes([cmd, target, zone])
        frame_wo_checksum = bytes([HEAD1, HEAD2, CMD_AUTO_TASK, LEN_AUTO_PAYLOAD]) + payload
        checksum = sum(frame_wo_checksum) & 0xFF
        return frame_wo_checksum + bytes([checksum])

    def _build_gait_packet(self, gait_id: int) -> bytes:
        """
        组装 0x11 串口协议帧
        格式：[0x55][0xAA][0x11][0x01][gait_id(1B)][checksum(1B)]
        """
        payload = bytes([gait_id & 0xFF])
        frame_wo_checksum = bytes([HEAD1, HEAD2, CMD_GAIT_SWITCH, 1]) + payload
        checksum = sum(frame_wo_checksum) & 0xFF
        return frame_wo_checksum + bytes([checksum])

    def _build_chassis_mode_packet(self, mode_id: int) -> bytes:
        """
        组装 0x14 串口协议帧（底盘模式切换）
        格式：[0x55][0xAA][0x14][0x01][mode_id(1B)][checksum(1B)]
        mode_id: 0=纯轮, 1=轮足, 2=纯足
        """
        payload = bytes([mode_id & 0xFF])
        frame_wo_checksum = bytes([HEAD1, HEAD2, CMD_CHASSIS_MODE, 1]) + payload
        checksum = sum(frame_wo_checksum) & 0xFF
        return frame_wo_checksum + bytes([checksum])

    def _send_tick(self):
        """
        定时发送任务（优先级：vision_cmd_vel > cmd_vel > stop）：
        1. 如果 vision_cmd_vel 在 vision_timeout 内收到且非零 → 使用视觉速度
        2. 否则如果 cmd_vel 在 stale_timeout 内收到 → 使用 Nav2 速度
        3. 否则 → 发送停止帧
        """
        now = time.monotonic()
        with self._lock:
            vision_age = now - self._vision_last_time if self._vision_last_time > 0 else self._vision_timeout + 1.0

            # 视觉速度仅在非零时有效（避免视觉停止后锁死 Nav2）
            use_vision = (
                vision_age < self._vision_timeout
                and self._vision_twist is not None
                and (abs(self._vision_twist.linear.x) >= _STATE_EPSILON
                     or abs(self._vision_twist.angular.z) >= _STATE_EPSILON)
            )

            if use_vision:
                vx = self._vision_twist.linear.x
                wz = self._vision_twist.angular.z
                state = derive_robot_state(vx, wz)
            else:
                twist = self._last_twist
                age = now - self._last_time if self._last_time > 0 else self._stale_timeout + 1.0

                if age > self._stale_timeout:
                    vx = wz = 0.0
                    state = ROBOT_STATE_IDLE
                else:
                    vx = twist.linear.x
                    wz = twist.angular.z
                    state = derive_robot_state(vx, wz)

        self.get_logger().info(f"发送数据: vx={vx:.3f} wz={wz:.3f}")
        pkt = self._build_packet(vx, wz, state)
        try:
            self._ser.write(pkt)
            self._ser.flush()
        except serial.SerialException as e:
            self.get_logger().error(f"Serial write failed: {e}")

    def destroy_node(self):
        """节点销毁时发送停止帧，确保机器人安全停车"""
        if self._zero_on_shutdown and self._ser and self._ser.is_open:
            try:
                self._ser.write(self._build_packet(0.0, 0.0, ROBOT_STATE_IDLE))
                self._ser.flush()
            except Exception:
                pass
            self._ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelChassisSerial()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
