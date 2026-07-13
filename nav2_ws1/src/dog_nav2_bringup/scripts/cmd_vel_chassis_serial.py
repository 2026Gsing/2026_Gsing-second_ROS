#!/usr/bin/env python3
"""
cmd_vel_chassis_serial.py — Nav2 /cmd_vel + Vision → STM32 串口协议转发

功能：
  1. 订阅 /cmd_vel (Nav2) → 发送 0x10 底盘速度帧（原有功能）
  2. 订阅 /vision_cmd_vel → 发送 0x10（视觉精细对位，优先级高于 Nav2）
  3. 订阅 /vision/auto_cmd → 发送 0x15 自动任务事件帧
  4. 订阅 /vision/arm_control → 发送 0x12 机械臂单次控制（x/y/z）
  5. 订阅 /vision/arm_mission → 发送 0x14 机械臂多段任务（pick/back/place）
  6. 超时保护：底盘命令超时自动停车

协议帧格式 0x10（底盘速度）：
  [0x55][0xAA][0x10][0x09][vx(f32)][wz(f32)][state(u8)][CheckSum]

协议帧格式 0x12（机械臂单次控制 FUNC_ARM_CONTROL）：
  [0x55][0xAA][0x12][0x0C][x(f32)][y(f32)][z(f32)][CheckSum]

协议帧格式 0x14（机械臂多段任务 FUNC_ARM_MISSION）：
  [0x55][0xAA][0x14][len][mode(u8)][flags(u8)][pick(12B)]...[back(12B)]...[place(12B)][CheckSum]

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
CMD_ARM_CONTROL = 0x12 # 功能码：机械臂单次控制
CMD_SUCTION     = 0x13 # 功能码：吸盘控制
CMD_ARM_MISSION = 0x14 # 功能码：机械臂多段任务（pick/back/place）
CMD_AUTO_TASK   = 0x15 # 功能码：自动任务事件
CMD_ARM_EVENT   = 0x22 # STM32 -> vision: arm mission event
LEN_VEL_PAYLOAD = 9    # 载荷长度：vx(4) + wz(4) + state(1) = 9
LEN_AUTO_PAYLOAD = 3   # 载荷长度：cmd(1) + target(1) + zone(1) = 3
LEN_ARM_EVENT_PAYLOAD = 16
PACKET_FMT = "<2fB"    # 打包格式（小端）：float32(vx), float32(wz), uint8(state)

ARM_EVENT_PICK_DONE = 0x01
ARM_EVENT_PLACE_DONE = 0x02
_ARM_EVENT_NAMES = {
    ARM_EVENT_PICK_DONE: "pick_done",
    ARM_EVENT_PLACE_DONE: "place_done",
}
_ARM_BACK_SIDE_NAMES = {
    0: "unknown",
    1: "left",
    2: "right",
    3: "center",
}

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
        self.declare_parameter("vision_state_topic", "/vision/robot_state")
        self.declare_parameter("auto_cmd_topic", "/vision/auto_cmd")
        self.declare_parameter("gait_cmd_topic", "/vision/gait_cmd")
        self.declare_parameter("arm_control_topic", "/vision/arm_control")
        self.declare_parameter("arm_mission_topic", "/vision/arm_mission")
        self.declare_parameter("arm_event_topic", "/vision/arm_event")
        self.declare_parameter("send_rate_hz", 50.0)
        self.declare_parameter("stale_timeout_sec", 0.25)
        self.declare_parameter("command_hold_timeout_sec", 0.75)
        self.declare_parameter("vision_timeout_sec", 0.5)   # 视觉控速超时
        self.declare_parameter("zero_on_shutdown", True)
        self.declare_parameter("log_period_sec", 1.0)
        self.declare_parameter("critical_repeat_count", 4)
        self.declare_parameter("active_state", 1)
        self.declare_parameter("idle_state", 0)

        # 读取参数
        port = self.get_parameter("serial_port").get_parameter_value().string_value
        baud = self.get_parameter("baud_rate").get_parameter_value().integer_value
        topic = self.get_parameter("cmd_vel_topic").get_parameter_value().string_value
        vision_topic = self.get_parameter("vision_cmd_vel_topic").get_parameter_value().string_value
        vision_state_topic = self.get_parameter("vision_state_topic").get_parameter_value().string_value
        auto_topic = self.get_parameter("auto_cmd_topic").get_parameter_value().string_value
        gait_topic = self.get_parameter("gait_cmd_topic").get_parameter_value().string_value
        arm_control_topic = self.get_parameter("arm_control_topic").get_parameter_value().string_value
        arm_mission_topic = self.get_parameter("arm_mission_topic").get_parameter_value().string_value
        arm_event_topic = self.get_parameter("arm_event_topic").get_parameter_value().string_value
        self._send_rate = max(
            10.0, self.get_parameter("send_rate_hz").get_parameter_value().double_value
        )
        self._stale_timeout = self.get_parameter("stale_timeout_sec").get_parameter_value().double_value
        self._command_hold_timeout = self.get_parameter("command_hold_timeout_sec").get_parameter_value().double_value
        if self._command_hold_timeout < self._stale_timeout:
            self._command_hold_timeout = self._stale_timeout
        self._vision_timeout = self.get_parameter("vision_timeout_sec").get_parameter_value().double_value
        self._log_period = self.get_parameter("log_period_sec").get_parameter_value().double_value
        self._critical_repeat_count = max(
            0, self.get_parameter("critical_repeat_count").get_parameter_value().integer_value
        )
        self._zero_on_shutdown = (
            self.get_parameter("zero_on_shutdown").get_parameter_value().bool_value
        )

        # ============ 状态变量 ============
        self._lock = threading.Lock()
        self._last_twist = Twist()
        self._last_time = 0.0
        self._vision_twist = None       # 视觉控速（可选，优先于 Nav2）
        self._vision_last_time = 0.0
        self._vision_state = None       # 视觉状态（从 /vision/robot_state 接收，可选）
        self._last_sent_source = "none"
        self._last_send_log_time = 0.0
        self._last_cmd_log_time = 0.0
        self._serial_lock = threading.Lock()
        self._critical_tx_queue = []
        self._rx_stop = threading.Event()
        self._rx_buffer = bytearray()

        # ============ 打开串口 ============
        try:
            self._ser = serial.Serial(port=port, baudrate=baud, timeout=0.05, write_timeout=0.02)
        except serial.SerialException as e:
            self.get_logger().error(f"串口 {port} 打开失败: {e}")
            self.get_logger().error("请检查: 1) STM32 是否已连接  2) sudo chmod 666 {port}")
            raise
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
        self.create_subscription(UInt8, vision_state_topic, self._vision_state_cb, _qos)
        self.create_subscription(String, auto_topic, self._auto_cmd_cb, _qos)
        self.create_subscription(UInt8, gait_topic, self._gait_cb, _qos)
        self.create_subscription(String, arm_control_topic, self._arm_control_cb, _qos)
        self.create_subscription(String, arm_mission_topic, self._arm_mission_cb, _qos)
        self.arm_event_pub = self.create_publisher(String, arm_event_topic, 10)
        self.get_logger().info(f"Publishing STM32 arm events on {arm_event_topic}")

        self._rx_thread = threading.Thread(target=self._serial_rx_loop, daemon=True)
        self._rx_thread.start()

        # ============ 定时发送 ============
        period = 1.0 / self._send_rate
        self._timer = self.create_timer(period, self._send_tick)

    def _write_serial(self, pkt: bytes, flush: bool = False):
        """Serialize all writes so chassis and arm frames never interleave."""
        with self._serial_lock:
            self._ser.write(pkt)
            if flush:
                self._ser.flush()

    def _queue_critical_packet(self, pkt: bytes, label: str):
        """Repeat non-periodic frames for a few ticks to survive a single USB/CDC loss."""
        if self._critical_repeat_count <= 0:
            return
        self._critical_tx_queue = [
            item for item in self._critical_tx_queue
            if item.get("label") != label
        ]
        self._critical_tx_queue.append({
            "packet": pkt,
            "remaining": self._critical_repeat_count,
            "label": label,
        })
        if len(self._critical_tx_queue) > 16:
            self._critical_tx_queue = self._critical_tx_queue[-16:]

    def _pop_critical_packet(self):
        if not self._critical_tx_queue:
            return None, None

        item = self._critical_tx_queue[0]
        pkt = item["packet"]
        label = item["label"]
        item["remaining"] -= 1
        if item["remaining"] <= 0:
            self._critical_tx_queue.pop(0)
        return pkt, label

    def _serial_rx_loop(self):
        while not self._rx_stop.is_set():
            try:
                chunk = self._ser.read(64)
            except (serial.SerialException, OSError) as e:
                if not self._rx_stop.is_set():
                    self.get_logger().error(f"Serial read failed: {e}")
                break
            if chunk:
                self._feed_rx_bytes(chunk)

    def _feed_rx_bytes(self, chunk: bytes):
        self._rx_buffer.extend(chunk)
        while True:
            if len(self._rx_buffer) < 5:
                return

            head = self._rx_buffer.find(bytes([HEAD1, HEAD2]))
            if head < 0:
                keep_last_head = len(self._rx_buffer) > 0 and self._rx_buffer[-1] == HEAD1
                self._rx_buffer[:] = bytes([HEAD1]) if keep_last_head else b""
                return
            if head > 0:
                del self._rx_buffer[:head]
            if len(self._rx_buffer) < 5:
                return

            payload_len = self._rx_buffer[3]
            frame_len = 5 + payload_len
            if len(self._rx_buffer) < frame_len:
                return

            frame = bytes(self._rx_buffer[:frame_len])
            del self._rx_buffer[:frame_len]

            checksum = sum(frame[:-1]) & 0xFF
            if checksum != frame[-1]:
                self.get_logger().warn(
                    f"Drop STM32 frame: checksum expected=0x{checksum:02X} got=0x{frame[-1]:02X}"
                )
                continue

            self._handle_rx_frame(frame[2], frame[4:-1])

    def _handle_rx_frame(self, func: int, payload: bytes):
        if func == CMD_ARM_EVENT:
            self._handle_arm_event(payload)

    def _handle_arm_event(self, payload: bytes):
        if len(payload) != LEN_ARM_EVENT_PAYLOAD:
            self.get_logger().warn(
                f"Drop arm event: len={len(payload)} expected={LEN_ARM_EVENT_PAYLOAD}"
            )
            return

        event, mission_mode, back_slot, back_side, x, y, z = struct.unpack("<BBBBfff", payload)
        event_name = _ARM_EVENT_NAMES.get(event, f"unknown_{event}")
        side_name = _ARM_BACK_SIDE_NAMES.get(back_side, f"unknown_{back_side}")
        data = {
            "event": event,
            "event_name": event_name,
            "mission_mode": mission_mode,
            "back_slot": back_slot,
            "back_side": back_side,
            "back_side_name": side_name,
            "target": {"x": x, "y": y, "z": z},
        }
        msg = String()
        msg.data = json.dumps(data, separators=(",", ":"))
        self.arm_event_pub.publish(msg)
        self.get_logger().info(
            f"[ARM_EVENT] {event_name} slot={back_slot} side={side_name} "
            f"target=({x:.3f},{y:.3f},{z:.3f})"
        )

    def _twist_cb(self, msg: Twist):
        """接收 Nav2 的 /cmd_vel 消息，缓存最新值"""
        now = time.monotonic()
        should_log = False
        with self._lock:
            self._last_twist = msg
            self._last_time = now
            if (now - self._last_cmd_log_time) >= self._log_period:
                should_log = True
                self._last_cmd_log_time = now
        if should_log:
            self.get_logger().info(
                f"收到 /cmd_vel: vx={msg.linear.x:.3f} wz={msg.angular.z:.3f}"
            )

    def _vision_cb(self, msg: Twist):
        """接收视觉 /vision_cmd_vel 消息（精细对位，覆盖 Nav2）"""
        with self._lock:
            self._vision_twist = msg
            self._vision_last_time = time.monotonic()

    def _vision_state_cb(self, msg: UInt8):
        """接收 /vision/robot_state 消息（test_move 等节点提供的状态，可选）"""
        with self._lock:
            self._vision_state = msg.data & 0xFF

    def _auto_cmd_cb(self, msg: String):
        """接收 /vision/auto_cmd JSON → 组装 0x15 帧发送"""
        try:
            data = json.loads(msg.data)
            cmd = int(data.get("cmd", 0)) & 0xFF
            target = int(data.get("target", 0)) & 0xFF
            zone = int(data.get("zone", 0)) & 0xFF
            pkt = self._build_auto_packet(cmd, target, zone)
            with self._lock:
                self._queue_critical_packet(pkt, f"0x15 cmd={cmd}")
            self._write_serial(pkt, flush=True)
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
                self._queue_critical_packet(pkt, f"0x11 gait={gait_id}")
            self._write_serial(pkt, flush=True)
            self.get_logger().info(f"[GAIT] 发送 0x11: gait_id={gait_id}")
        except Exception as e:
            self.get_logger().error(f"[GAIT] 发送失败: {e}")

    def _arm_control_cb(self, msg: String):
        """
        接收 /vision/arm_control JSON → 组装 0x12 帧发送（FUNC_ARM_CONTROL）

        STM32 协议格式（与 protocol_handler.c FUNC_ARM_CONTROL 匹配）：
          [0x55][0xAA][0x12][0x0C][x(f32)][y(f32)][z(f32)][checksum]

        JSON 格式：
          {"x": 0.1, "y": 0.2, "z": -0.15}
        """
        try:
            data = json.loads(msg.data)
            x = float(data.get("x", 0.0))
            y = float(data.get("y", 0.0))
            z = float(data.get("z", 0.0))
            pkt = self._build_arm_control_packet(x, y, z)
            with self._lock:
                self._queue_critical_packet(pkt, "0x12 arm_control")
            self._write_serial(pkt, flush=True)
            # 节流日志：坐标变化或 5s 没打印才输出
            now = time.monotonic()
            if (x, y, z) != getattr(self, '_last_arm_log_xyz', None) or \
               (now - getattr(self, '_last_arm_log_time', 0.0)) >= 5.0:
                self._last_arm_log_xyz = (x, y, z)
                self._last_arm_log_time = now
                self.get_logger().info(
                    f"[ARM] 发送 0x12: x={x:.3f} y={y:.3f} z={z:.3f}"
                )
        except Exception as e:
            self.get_logger().error(f"[ARM] 解析/发送失败: {e}")

    def _arm_mission_cb(self, msg: String):
        """
        接收 /vision/arm_mission JSON → 组装 0x14 帧发送（FUNC_ARM_MISSION）

        STM32 协议格式（与 protocol_handler.c FUNC_ARM_MISSION 匹配）：
          [0x55][0xAA][0x14][len][mode(u8)][flags(u8)][pick(xyz=12B)]...[back(xyz=12B)]...[place(xyz=12B)][checksum]

        flags 位：0x01=HAS_PICK, 0x02=HAS_BACK, 0x04=HAS_PLACE

        JSON 格式示例：
          {"mode":1, "flags":7,
           "pick":[0.1,0.2,0.3], "back":[0.15,0.25,0.1], "place":[0.2,0.1,0.05]}

          {"mode":1, "flags":1, "pick":[0.1,0.2,0.3]}  # 仅 pick
        """
        try:
            data = json.loads(msg.data)
            mode = int(data.get("mode", 0)) & 0xFF
            flags = int(data.get("flags", 0)) & 0xFF

            payload = bytes([mode, flags])

            for key in ("pick", "back", "place"):
                if key in data:
                    xyz = data[key]
                    payload += struct.pack("<3f",
                        float(xyz[0]), float(xyz[1]), float(xyz[2]))

            pkt = self._build_arm_mission_packet(payload)
            with self._lock:
                self._queue_critical_packet(pkt, f"0x14 arm_mission mode={mode}")
            self._write_serial(pkt, flush=True)
            self.get_logger().info(
                f"[ARM_MISSION] 发送 0x14: mode={mode} flags={flags} "
                f"len={len(payload)}B"
            )
        except Exception as e:
            self.get_logger().error(f"[ARM_MISSION] 解析/发送失败: {e}")

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

    def _build_arm_control_packet(self, x: float, y: float, z: float) -> bytes:
        """
        组装 0x12 串口协议帧（FUNC_ARM_CONTROL）
        格式：[0x55][0xAA][0x12][0x0C][x(f32)][y(f32)][z(f32)][checksum]
        """
        payload = struct.pack("<3f", float(x), float(y), float(z))
        frame_wo_checksum = bytes([HEAD1, HEAD2, CMD_ARM_CONTROL, 12]) + payload
        checksum = sum(frame_wo_checksum) & 0xFF
        return frame_wo_checksum + bytes([checksum])

    def _build_arm_mission_packet(self, payload: bytes) -> bytes:
        """
        组装 0x14 串口协议帧（FUNC_ARM_MISSION）
        格式：[0x55][0xAA][0x14][len][payload...][checksum]
        """
        frame_wo_checksum = bytes([HEAD1, HEAD2, CMD_ARM_MISSION, len(payload)]) + payload
        checksum = sum(frame_wo_checksum) & 0xFF
        return frame_wo_checksum + bytes([checksum])

    def _send_tick(self):
        """
        定时发送任务（优先级：vision_cmd_vel > cmd_vel > stop）：
        1. 如果 vision_cmd_vel 在 vision_timeout 内收到且非零 → 使用视觉速度
        2. 否则如果 cmd_vel 在 stale_timeout 内收到 → 使用 Nav2 速度
        3. 否则短时间保持上一条非零 cmd_vel，滤掉 ROS 调度抖动
        4. 超过 command_hold_timeout → 发送停止帧
        """
        now = time.monotonic()
        with self._lock:
            vision_age = now - self._vision_last_time if self._vision_last_time > 0 else self._vision_timeout + 1.0
            source = "stop"

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
                source = "vision"
                # 优先使用 /vision/robot_state 提供的状态（如 test_move 推导的）
                if self._vision_state is not None:
                    state = self._vision_state
                else:
                    state = derive_robot_state(vx, wz)
            else:
                twist = self._last_twist
                age = now - self._last_time if self._last_time > 0 else self._stale_timeout + 1.0

                twist_is_nonzero = (
                    abs(twist.linear.x) >= _STATE_EPSILON or
                    abs(twist.angular.z) >= _STATE_EPSILON
                )

                if age <= self._stale_timeout:
                    vx = twist.linear.x
                    wz = twist.angular.z
                    state = derive_robot_state(vx, wz)
                    source = "cmd_vel"
                elif age <= self._command_hold_timeout and twist_is_nonzero:
                    vx = twist.linear.x
                    wz = twist.angular.z
                    state = derive_robot_state(vx, wz)
                    source = "cmd_vel_hold"
                else:
                    vx = wz = 0.0
                    state = ROBOT_STATE_IDLE
                    source = "stop"

            transition = source != self._last_sent_source
            should_log = False
            if transition:
                should_log = True
                self._last_sent_source = source
            elif (now - self._last_send_log_time) >= self._log_period and abs(vx) + abs(wz) > 1e-6:
                should_log = True
            if should_log:
                self._last_send_log_time = now
            critical_pkt, critical_label = self._pop_critical_packet()

        if should_log and (abs(vx) + abs(wz) > 1e-6 or transition):
            self.get_logger().info(f"send source={source} vx={vx:.3f} wz={wz:.3f}")
        pkt = self._build_packet(vx, wz, state)
        try:
            self._write_serial(pkt)
            if critical_pkt is not None:
                self._write_serial(critical_pkt, flush=True)
                self.get_logger().debug(f"repeat {critical_label}")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial write failed: {e}")

    def destroy_node(self):
        """节点销毁时发送停止帧，确保机器人安全停车"""
        self._rx_stop.set()
        if self._zero_on_shutdown and self._ser and self._ser.is_open:
            try:
                self._write_serial(self._build_packet(0.0, 0.0, ROBOT_STATE_IDLE), flush=True)
            except Exception:
                pass
        if self._ser and self._ser.is_open:
            try:
                self._ser.close()
            except Exception:
                pass
        if getattr(self, "_rx_thread", None) and self._rx_thread.is_alive():
            self._rx_thread.join(timeout=0.2)
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
