#!/usr/bin/env python3
"""
Chassis movement bridge: Nav2 /cmd_vel -> STM32 serial protocol.

Protocol frame (per your doc):
  [Head1=0x55][Head2=0xAA][FuncID][Len][Payload...][CheckSum]
  - FuncID for chassis velocity: 0x10
  - Len: 9
  - Payload: vx(float32), wz(float32), state(uint8)
  - CheckSum: low 8-bit of sum(Byte0 ... Byte(4+Len-1))
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

HEAD1 = 0x55
HEAD2 = 0xAA
CMD_CHASSIS_VEL = 0x10
LEN_VEL_PAYLOAD = 9
PACKET_FMT = "<2fB"  # little-endian: vx(float), wz(float), state(uint8)


class CmdVelChassisSerial(Node):
    def __init__(self):
        super().__init__("cmd_vel_chassis_serial")

        if serial is None:
            raise RuntimeError(
                "需要安装 pyserial: sudo apt install python3-serial"
            ) from _SERIAL_IMPORT_ERROR

        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("send_rate_hz", 50.0)
        self.declare_parameter("stale_timeout_sec", 0.08)
        self.declare_parameter("zero_on_shutdown", True)
        self.declare_parameter("active_state", 1)
        self.declare_parameter("idle_state", 0)

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

        self._lock = threading.Lock()
        self._last_twist = Twist()
        self._last_time = 0.0

        self._ser = serial.Serial(port=port, baudrate=baud, timeout=0.05)
        self.get_logger().info(
            f"Opened {port} @ {baud}, cmd_vel={topic}, send={self._send_rate}Hz, "
            f"states(active={self._active_state}, idle={self._idle_state})"
        )

        self.create_subscription(Twist, topic, self._twist_cb, 10)

        period = 1.0 / self._send_rate
        self._timer = self.create_timer(period, self._send_tick)

    def _twist_cb(self, msg: Twist):
        with self._lock:
            self._last_twist = msg
            self._last_time = time.monotonic()

    def _build_packet(self, vx: float, wz: float, state: int) -> bytes:
        """Frame: [55][AA][FuncID][Len][9B payload][Checksum]."""
        payload = struct.pack(PACKET_FMT, float(vx), float(wz), int(state) & 0xFF)
        frame_wo_checksum = bytes([HEAD1, HEAD2, CMD_CHASSIS_VEL, LEN_VEL_PAYLOAD]) + payload
        checksum = sum(frame_wo_checksum) & 0xFF
        return frame_wo_checksum + bytes([checksum])

    def _send_tick(self):
        now = time.monotonic()
        with self._lock:
            twist = self._last_twist
            age = now - self._last_time if self._last_time > 0 else self._stale_timeout + 1.0

        if age > self._stale_timeout:
            vx = wz = 0.0
            state = self._idle_state
        else:
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
