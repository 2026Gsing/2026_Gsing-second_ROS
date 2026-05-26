#!/usr/bin/env python3
"""
cmd_vel_chassis_serial.py — Nav2 /cmd_vel → STM32 串口协议转发

功能：
  订阅 ROS2 /cmd_vel (Twist) 消息，按照底盘通信协议打包成帧，
  通过串口发送给下位机 STM32，实现机器人底盘运动控制。

协议帧格式：
  [0x55][0xAA][0x10][0x09][vx(float32)][wz(float32)][state(uint8)][CheckSum]
  - vx: 线速度 (m/s)
  - wz: 角速度 (rad/s)
  - state: 状态字节（1=主动控制模式, 0=空闲/停止）
  - CheckSum: 前面所有字节的和的低8位

安全机制：
  1. 超时保护 (stale_timeout): 超过 80ms 未收到新 cmd_vel 时自动发送停止帧
  2. 退出保护: 节点销毁时发送停止帧，防止机器人继续运动
  3. 最低发送频率: send_rate_hz 最低 clamp 到 10Hz（对应 STM32 100ms 看门狗）
"""

import struct
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

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
LEN_VEL_PAYLOAD = 9    # 载荷长度：vx(4) + wz(4) + state(1) = 9
PACKET_FMT = "<2fB"    # 打包格式（小端）：float32(vx), float32(wz), uint8(state)


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
        self.declare_parameter("send_rate_hz", 50.0)
        self.declare_parameter("stale_timeout_sec", 0.08)
        self.declare_parameter("zero_on_shutdown", True)
        self.declare_parameter("active_state", 1)
        self.declare_parameter("idle_state", 0)

        # 读取参数
        port = self.get_parameter("serial_port").get_parameter_value().string_value
        baud = self.get_parameter("baud_rate").get_parameter_value().integer_value
        topic = self.get_parameter("cmd_vel_topic").get_parameter_value().string_value
        self._send_rate = max(
            10.0, self.get_parameter("send_rate_hz").get_parameter_value().double_value
        )
        self._stale_timeout = self.get_parameter("stale_timeout_sec").get_parameter_value().double_value
        self._zero_on_shutdown = (
            self.get_parameter("zero_on_shutdown").get_parameter_value().bool_value
        )
        self._active_state = int(
            self.get_parameter("active_state").get_parameter_value().integer_value
        ) & 0xFF
        self._idle_state = int(
            self.get_parameter("idle_state").get_parameter_value().integer_value
        ) & 0xFF

        # ============ 状态变量 ============
        self._lock = threading.Lock()        # 线程锁，保护共享数据
        self._last_twist = Twist()           # 最近一次收到的速度命令
        self._last_time = 0.0                # 最近一次收到命令的时间戳

        # ============ 打开串口 ============
        self._ser = serial.Serial(port=port, baudrate=baud, timeout=0.05)
        self.get_logger().info(
            f"Opened {port} @ {baud}, cmd_vel={topic}, send={self._send_rate}Hz, "
            f"states(active={self._active_state}, idle={self._idle_state})"
        )

        # ============ 订阅 /cmd_vel ============
        self.create_subscription(Twist, topic, self._twist_cb, 10)

        # ============ 定时发送 ============
        period = 1.0 / self._send_rate
        self._timer = self.create_timer(period, self._send_tick)

    def _twist_cb(self, msg: Twist):
        """接收 Nav2 的 /cmd_vel 消息，缓存最新值"""
        with self._lock:
            self._last_twist = msg
            self._last_time = time.monotonic()

    def _build_packet(self, vx: float, wz: float, state: int) -> bytes:
        """
        组装串口协议帧
        格式：[0x55][0xAA][0x10][0x09][vx(4B)][wz(4B)][state(1B)][checksum(1B)]
        """
        payload = struct.pack(PACKET_FMT, float(vx), float(wz), int(state) & 0xFF)
        frame_wo_checksum = bytes([HEAD1, HEAD2, CMD_CHASSIS_VEL, LEN_VEL_PAYLOAD]) + payload
        checksum = sum(frame_wo_checksum) & 0xFF
        return frame_wo_checksum + bytes([checksum])

    def _send_tick(self):
        """
        定时发送任务：
        - 如果在 stale_timeout 内收到新命令 → 发送 active_state + 速度值
        - 如果超时 → 发送 idle_state + 零速度（停止）
        """
        now = time.monotonic()
        with self._lock:
            twist = self._last_twist
            age = now - self._last_time if self._last_time > 0 else self._stale_timeout + 1.0

        if age > self._stale_timeout:
            # 命令超时 → 发送停止帧
            vx = wz = 0.0
            state = self._idle_state
        else:
            # 命令有效 → 发送控制帧
            vx = twist.linear.x
            wz = twist.angular.z
            state = self._active_state

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
                self._ser.write(self._build_packet(0.0, 0.0, self._idle_state))
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
