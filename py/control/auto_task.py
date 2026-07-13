#!/usr/bin/env python3
"""
vision_auto_task_node.py — 视觉全自动任务执行节点

职责：
  1. 视觉状态机 IDLE → SOLVE_TASK → NAV_BOX → WAIT_PICK
                        → NAV_ZONE → WAIT_PLACE → NEXT_OR_FINISH (loop)
  2. YOLO 物资箱检测 + 数学题识别（可选，可通过文件或 ROS topic 输入）
  3. 发送 Nav2 导航目标
  4. 到达检测（订阅 /localization）
  5. 发布自动任务命令到 /vision/auto_cmd（由 cmd_vel_chassis_serial.py 转发 0x15）
  6. 发布精细控速到 /vision_cmd_vel

使用方式：
  python3 py/control/auto_task.py

依赖：
  rclpy, nav2_msgs, geometry_msgs, nav_msgs, std_msgs, pyyaml
"""

import math
import sys
import time
import json
import subprocess
import os
import signal
import atexit
from pathlib import Path

from launch_utils import launch, cleanup_all

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import yaml

from arrival_detector import ArrivalDetector, quaternion_to_yaw, normalize_angle
from visualization_msgs.msg import Marker

# 可选导入：cube_detector + catch 模块
try:
    from cube_detector import CubeDetector as _CubeDetector
    _HAVE_CUBE_DETECTOR = True
except Exception:
    _CubeDetector = None
    _HAVE_CUBE_DETECTOR = False

try:
    from catch import (
        transform_and_offset, find_stable_points, workspace_check,
        OFFSET_X, OFFSET_Z, HALF_BOX_HEIGHT,
        ARM_WORKSPACE_RADIUS_MIN, ARM_WORKSPACE_RADIUS_MAX,
        STABLE_THRESHOLD_XY, STABLE_COUNT_REQUIRED, CACHE_MAX_SIZE,
    )
    _HAVE_CATCH = True
except Exception:
    _HAVE_CATCH = False

# ╔══════════════════════════════════════════════════════════╗
# ║  赛前配置 — 改 py/config/competition.yaml               ║
# ╚══════════════════════════════════════════════════════════╝
# field_id:=1 或 field_id:=2（启动时命令行指定，也可改 yaml）

# 从配置文件加载（见 py/config/competition.yaml）
_CFG = {}
_cfg_path = Path(__file__).resolve().parent.parent.parent / "config" / "competition.yaml"
if _cfg_path.exists():
    try:
        _CFG = yaml.safe_load(_cfg_path.read_text(encoding="utf-8"))
    except Exception:
        _CFG = {}

FIELD_BOXES = _CFG.get("field_boxes", {
    1: [[1, 2, 3, 4], [0, 0, 0, 0]],
    2: [[4, 3, 2, 1], [0, 0, 0, 0]],
})
FIELD_ZONES = _CFG.get("field_zones", {1: [1, 2, 3, 4], 2: [4, 3, 2, 1]})

_coord = _CFG.get("coordinates", {})
_BOX_COL_X = _coord.get("box_col_x", [0.5, 1.1, 1.7, 2.3])
_BOX_OUTER_Y = _coord.get("box_outer_y", 1.0)
_BOX_INNER_Y = _coord.get("box_inner_y", 1.5)
_BOX_YAW = _coord.get("box_yaw", 0.0)
_ZONE_X = _coord.get("zone_x", [0.6, 1.4, 2.2, 3.0])
_ZONE_Y = _coord.get("zone_y", 5.0)
_ZONE_YAW = _coord.get("zone_yaw", 3.14)

_BOX_TYPE_NAMES = _CFG.get("box_type_names", {1: "食品", 2: "工具", 3: "仪器", 4: "药品"})

# ============================================================
# AUTO_CMD 常量（与 STM32 protocol_handler.h 严格一致）
# ============================================================
AUTO_CMD_NONE = 0
AUTO_CMD_START = 1
AUTO_CMD_ARRIVED_BOX = 2
AUTO_CMD_PICK_DONE = 3
AUTO_CMD_ARRIVED_ZONE = 4
AUTO_CMD_PLACE_DONE = 5
AUTO_CMD_NEXT = 6
AUTO_CMD_FINISH = 7
AUTO_CMD_ESTOP = 8

_CMD_NAMES = {
    AUTO_CMD_NONE: "NONE", AUTO_CMD_START: "START",
    AUTO_CMD_ARRIVED_BOX: "ARRIVED_BOX", AUTO_CMD_PICK_DONE: "PICK_DONE",
    AUTO_CMD_ARRIVED_ZONE: "ARRIVED_ZONE", AUTO_CMD_PLACE_DONE: "PLACE_DONE",
    AUTO_CMD_NEXT: "NEXT", AUTO_CMD_FINISH: "FINISH", AUTO_CMD_ESTOP: "ESTOP",
}

# ============================================================
# 视觉状态枚举
# ============================================================
class VS:
    IDLE = 0
    SOLVE_TASK = 1
    FIND_BOX = 2
    NAV_BOX = 3
    WAIT_PICK = 4
    NAV_ZONE = 5
    WAIT_PLACE = 6
    NEXT_OR_FINISH = 7
    ERROR = 8

_STATE_NAMES = {
    VS.IDLE: "IDLE", VS.SOLVE_TASK: "SOLVE_TASK", VS.FIND_BOX: "FIND_BOX",
    VS.NAV_BOX: "NAV_BOX", VS.WAIT_PICK: "WAIT_PICK", VS.NAV_ZONE: "NAV_ZONE",
    VS.WAIT_PLACE: "WAIT_PLACE", VS.NEXT_OR_FINISH: "NEXT_OR_FINISH",
    VS.ERROR: "ERROR",
}

# 当前工作区路径
_HERE = Path(__file__).resolve().parent          # py/control/
_PROJECT = _HERE.parent.parent                   # 2026_Gsing-second_ROS/
_VISION_DIR = _PROJECT / "vision"                # YOLO 代码整合目录
_YOLO_SRC = _VISION_DIR / "src"
_VISION_WEIGHTS = _VISION_DIR / "weights"
_VISION_CONFIG = _VISION_DIR / "config"

class VisionAutoTaskNode(Node):
    def __init__(self):
        super().__init__("vision_auto_task")

        # ======================== 参数（默认值来自 competition.yaml）=======================
        _to = _CFG.get("timeouts", {})
        self.declare_parameter("arrival_pos_threshold", _to.get("nav2_arrival_pos", 0.25))
        self.declare_parameter("arrival_angle_threshold", _to.get("nav2_arrival_angle", 0.30))
        self.declare_parameter("arrival_settle_frames", _to.get("nav2_arrival_frames", 5))
        self.declare_parameter("pick_timeout_sec", _to.get("pick", 5.0))
        self.declare_parameter("place_timeout_sec", _to.get("place", 5.0))
        self.declare_parameter("total_boxes", _CFG.get("total_boxes", 8))
        self.declare_parameter("yolo_decision_file", str(_VISION_CONFIG / "decision_state.json"))
        self.declare_parameter("nav2_action_timeout_sec", _to.get("nav2_action", 30.0))
        self.declare_parameter("competition_timeout_sec", _to.get("competition", 180.0))
        self.declare_parameter("field_id", _CFG.get("field_id", 1))

        # ======================== 状态机 ========================
        self.state = VS.IDLE
        self.field_id = self.get_parameter("field_id").value
        self.current_step_idx = 0
        self.total_boxes = self.get_parameter("total_boxes").value
        self.pickup_sequence = []
        self.zone_sequence = []
        self.high_score_zone = None
        self.target_class = None
        self.math_timeout_sec = _to.get("math", 20.0)
        self.competition_start_time = 0.0
        self._last_time_log = 0.0
        self.arrival = ArrivalDetector(
            position_threshold=self.get_parameter("arrival_pos_threshold").value,
            angle_threshold=self.get_parameter("arrival_angle_threshold").value,
            settle_frames=self.get_parameter("arrival_settle_frames").value,
        )
        self._state_enter_time = 0.0  # 进入当前状态的时间戳
        self._wait_ok = False         # 等待阶段的完成标志

        # ======================== 导航 ========================
        self._cb_group = ReentrantCallbackGroup()
        self.nav_client = ActionClient(
            self, NavigateToPose, "navigate_to_pose",
            callback_group=self._cb_group,
        )
        self._nav_goal_handle = None
        self._nav_result_future = None
        self._nav_succeeded = False

        # ======================== 加载目标点 ========================
        wp_file = self.get_parameter("waypoints_file").value
        self.waypoints = self._load_waypoints(wp_file)
        self.get_logger().info(
            f"加载目标点: {len(self.waypoints.get('boxes',[]))} 箱, "
            f"{len(self.waypoints.get('zones',[]))} 区"
        )

        # ======================== 订阅 ========================
        self.create_subscription(Odometry, "/localization", self._localization_cb, 10)
        # 可选：订阅 YOLO 检测结果（topic 方式）
        self.create_subscription(String, "/vision/math_result", self._math_result_cb, 10)
        self.create_subscription(String, "/vision/detected_objects", self._detection_cb, 10)
        # 订阅 LiDAR 立方体检测结果（来自 cube_detector.py）
        self.create_subscription(Marker, "/detected_cube", self._cube_cb, 10)

        # ======================== 发布 ========================
        # auto_cmd_payload: JSON {"cmd":2,"target":0,"zone":1} 发给 serial 桥
        self.auto_cmd_pub = self.create_publisher(String, "/vision/auto_cmd", 10)
        # 机械臂坐标（由串口桥转发 0x12）
        self.arm_pub = self.create_publisher(String, "/vision/arm_control", 10)
        # 精细对位速度（替代 Nav2 的最后一段）
        self.vel_pub = self.create_publisher(Twist, "/vision_cmd_vel", 10)
        # 状态发布
        self.state_pub = self.create_publisher(String, "/vision/auto_task_state", 10)

        # ======================== 定时器 (20Hz) ========================
        self.create_timer(0.05, self._tick)
        # 中等频率 YOLO 决策更新 (5Hz)
        self.create_timer(0.20, self._yolo_tick)

        # ======================== YOLO 检测数据 ========================
        self._yolo_math_result = None   # 最近一次数学题结果
        self._yolo_detections = {}      # 最近一次检测结果 {slot_label: class_name}
        self._decision_file = Path(self.get_parameter("yolo_decision_file").value)
        self._yolo_available = False
        self._init_yolo()

        # ======================== 立方体检测数据 ========================
        self._latest_cube = None          # 最新的 /detected_cube (Marker)
        self._cube_recv_time = 0.0        # 时间戳
        self._cube_cache = []             # 稳定检测滑动窗口
        self._pick_done = False           # 抓取完成标记

        self.get_logger().info("VisionAutoTaskNode 已启动，等待 START 命令")
        self._publish_state()

    # ================================================================
    # YOLO 初始化（可选）
    # ================================================================
    def _init_yolo(self):
        """尝试导入 YOLO，失败则降级为文件读取"""
        # 尝试将 YOLO src 加入路径
        if str(_YOLO_SRC) not in sys.path:
            sys.path.insert(0, str(_YOLO_SRC))
        try:
            from ultralytics import YOLO
            task_w = _VISION_WEIGHTS / "task3.pt"
            math_w = _VISION_WEIGHTS / "math12.pt"

            self._yolo_task_model = YOLO(str(task_w)) if task_w.exists() else None
            self._yolo_math_model = YOLO(str(math_w)) if math_w.exists() else None

            if self._yolo_task_model or self._yolo_math_model:
                self._yolo_available = True
                self.get_logger().info(
                    f"YOLO 加载: task={'✓' if self._yolo_task_model else '✗'}, "
                    f"math={'✓' if self._yolo_math_model else '✗'}"
                )
            else:
                self.get_logger().warn("YOLO 权重文件未找到，降级为文件输入模式")
        except Exception as e:
            self.get_logger().warn(f"YOLO 导入失败 ({e})，降级为文件输入模式")

    # ================================================================
    # 目标点加载
    # ================================================================
    def _load_waypoints(self, path):
        try:
            p = Path(path)
            if p.exists():
                with open(p) as f:
                    return yaml.safe_load(f)
        except Exception as e:
            self.get_logger().warn(f"加载 {path} 失败: {e}")
        # 默认值（根据 task_field_map 坐标推算，后续可调）
        return {
            "boxes": [
                {"id": 0, "x": 0.5, "y": 1.0, "yaw": 0.0},
                {"id": 1, "x": 1.1, "y": 1.0, "yaw": 0.0},
                {"id": 2, "x": 1.7, "y": 1.0, "yaw": 0.0},
                {"id": 3, "x": 2.3, "y": 1.0, "yaw": 0.0},
                {"id": 4, "x": 0.5, "y": 1.5, "yaw": 0.0},
                {"id": 5, "x": 1.1, "y": 1.5, "yaw": 0.0},
                {"id": 6, "x": 1.7, "y": 1.5, "yaw": 0.0},
                {"id": 7, "x": 2.3, "y": 1.5, "yaw": 0.0},
            ],
            "zones": [
                {"id": 0, "x": 0.6, "y": 5.0, "yaw": 3.14},
                {"id": 1, "x": 1.4, "y": 5.0, "yaw": 3.14},
                {"id": 2, "x": 2.2, "y": 5.0, "yaw": 3.14},
                {"id": 3, "x": 3.0, "y": 5.0, "yaw": 3.14},
            ],
        }

    # ================================================================
    # 回调
    # ================================================================
    def _localization_cb(self, msg):
        """更新最新位姿（用于到达检测）"""
        self.arrival.check(msg.pose.pose)  # 喂给到达检测器

    def _math_result_cb(self, msg):
        """接收 YOLO 数学题结果 (ROS topic)"""
        try:
            self._yolo_math_result = json.loads(msg.data)
        except Exception:
            pass

    def _detection_cb(self, msg):
        """接收 YOLO 检测结果 (ROS topic)"""
        try:
            data = json.loads(msg.data)
            if isinstance(data, dict):
                self._yolo_detections = data
        except Exception:
            pass

    def _cube_cb(self, msg):
        """接收 cube_detector 的立方体检测结果"""
        self._latest_cube = msg
        self._cube_recv_time = time.monotonic()

    def _read_decision_file(self):
        """从 JSON 文件读取数学决策结果（文件 IPC 方式）"""
        if self._decision_file.exists():
            try:
                return json.loads(self._decision_file.read_text(encoding="utf-8"))
            except Exception:
                pass
        return None

    def _read_nav_target_file(self):
        """从 JSON 文件读取导航目标（文件 IPC 方式）"""
        p = self._decision_file.parent / "nav_target.json"
        if p.exists():
            try:
                return json.loads(p.read_text(encoding="utf-8"))
            except Exception:
                pass
        return None

    # ================================================================
    # 命令发送
    # ================================================================
    def send_auto_cmd(self, command: int, target_id: int = 0, zone_id: int = 0):
        """发送 0x15 自动任务命令（JSON → /vision/auto_cmd）"""
        payload = json.dumps({
            "cmd": command & 0xFF,
            "target": target_id & 0xFF,
            "zone": zone_id & 0xFF,
        })
        msg = String()
        msg.data = payload
        self.auto_cmd_pub.publish(msg)
        name = _CMD_NAMES.get(command, f"0x{command:02X}")
        self.get_logger().info(f"[AUTO] {name} target={target_id} zone={zone_id}")

    def send_nav2_goal(self, x: float, y: float, yaw: float):
        """发送 Nav2 导航目标"""
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("Nav2 action server 不可用")
            return False

        self._nav_succeeded = False
        send_future = self.nav_client.send_goal_async(goal)
        send_future.add_done_callback(self._nav_goal_response_cb)
        self.get_logger().info(f"[NAV] 发送目标: ({x:.2f}, {y:.2f}, {yaw:.2f})")
        return True

    def _nav_goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("[NAV] 目标被拒绝")
            return
        self._nav_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav_result_cb)

    def _nav_result_cb(self, future):
        result = future.result()
        if result and result.status == 4:  # SUCCEEDED
            self._nav_succeeded = True
            self.get_logger().info("[NAV] 导航成功！")
        else:
            status = result.status if result else -1
            self.get_logger().warn(f"[NAV] 导航结束, status={status}")

    def send_velocity(self, vx: float, wz: float):
        """发布精细控速（由扩展串口桥转发 0x10）"""
        msg = Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(wz)
        self.vel_pub.publish(msg)

    def send_arm_target(self, x: float, y: float, z: float):
        """发布机械臂坐标到 /vision/arm_control（串口桥转发 0x12）"""
        msg = String()
        msg.data = json.dumps({"x": x, "y": y, "z": z})
        self.arm_pub.publish(msg)
        self.get_logger().info(f"[ARM] 坐标 ({x:.3f}, {y:.3f}, {z:.3f})")

    def _publish_state(self):
        msg = String()
        msg.data = _STATE_NAMES.get(self.state, "UNKNOWN")
        self.state_pub.publish(msg)

    def stop_all(self):
        """紧急停止：停止导航 + 清零速度 + 急停命令"""
        if self._nav_goal_handle:
            self._nav_goal_handle.cancel_goal_async()
        self.send_velocity(0.0, 0.0)
        self.send_auto_cmd(AUTO_CMD_ESTOP)
        self.state = VS.ERROR



    # ================================================================
    # 状态入口
    # ================================================================
    def _transition_to(self, new_state):
        old = self.state
        self.state = new_state
        self._state_enter_time = time.monotonic()
        self._wait_ok = False
        self._publish_state()
        self.get_logger().info(
            f"[状态] {_STATE_NAMES.get(old,'?')} → {_STATE_NAMES.get(new_state,'?')}"
        )

        # 进入 WAIT_PICK/WAIT_PLACE 时重置检测缓存
        if new_state in (VS.WAIT_PICK, VS.WAIT_PLACE):
            self._cube_cache = []
            self._pick_done = False
            self._latest_cube = None

    def _elapsed(self) -> float:
        return time.monotonic() - self._state_enter_time

    def _generate_pickup_sequence(self, mod4):
        """
        根据场地号和数学题结果生成最优拾取序列。
        返回: [(box_type, zone_id, box_x, box_y, box_yaw, zone_x, zone_y, zone_yaw), ...]
        策略: 高分区类型优先 → 外排(近启动区)优先 → 左到右
        """
        field = self.field_id
        boxes = FIELD_BOXES.get(field)
        zones = FIELD_ZONES.get(field)
        if boxes is None or zones is None:
            self.get_logger().error(f"未知场地号 {field}")
            return []

        # 高分区类型: mod4 → zone_idx → 对应的 box_type
        if mod4 is not None:
            high_zone_idx = mod4 % 4
            high_type = zones[high_zone_idx]
        else:
            high_type = None

        # 构建 (box_type, row, col) 列表
        all_boxes = []
        for row in range(2):          # 0=外排(靠启动区), 1=内排(靠归位区)
            for col in range(4):
                bt = boxes[row][col]
                if bt == 0:
                    continue
                y = _BOX_OUTER_Y if row == 0 else _BOX_INNER_Y
                all_boxes.append((bt, _BOX_COL_X[col], y, _BOX_YAW))

        # 排序: 高分区类型优先, 近启动区优先, 左到右
        def sort_key(item):
            bt, bx, by, byaw = item
            is_high = 0 if bt == high_type else 1  # high type first
            is_outer = 0 if by == _BOX_OUTER_Y else 1  # outer (near launch) first
            return (is_high, is_outer, bx)

        all_boxes.sort(key=sort_key)

        # 构建完整序列: 每箱 → 其匹配区
        sequence = []
        for bt, bx, by, byaw in all_boxes:
            # 找到此箱类型对应的区索引
            try:
                zone_idx = zones.index(bt)
            except ValueError:
                self.get_logger().error(f"箱类型 {bt} 无对应区")
                continue
            zx = _ZONE_X[zone_idx]
            sequence.append((bt, zone_idx, bx, by, byaw, zx, _ZONE_Y, _ZONE_YAW))

        self.get_logger().info(f"[场地 {field}] 拾取序列 ({len(sequence)} 箱):")
        for i, (bt, zi, bx, by, _, zx, zy, _) in enumerate(sequence):
            high = "★" if bt == high_type else " "
            self.get_logger().info(
                f"  [{i}] {high}{_BOX_TYPE_NAMES.get(bt, '?')} "
                f"箱({bx:.1f},{by:.1f}) → 区{zi}({zx:.1f},{zy:.1f})"
            )

        return sequence

    # ================================================================
    # YOLO 决策 tick (5Hz)
    # ================================================================
    def _yolo_tick(self):
        """定期读取 YOLO 决策结果（文件或 ROS topic）"""
        # 优先使用 ROS topic 数据，其次文件
        if self._yolo_math_result is None:
            self._yolo_math_result = self._read_decision_file()
        if not self._yolo_detections:
            nav = self._read_nav_target_file()
            if nav and nav.get("slot_id"):
                self._yolo_detections = nav

    # ================================================================
    # 主状态机 tick (20Hz)
    # ================================================================
    def _tick(self):
        try:
            # 比赛总时长检查（手册: 180秒）
            if self.state not in (VS.IDLE, VS.ERROR):
                self._check_competition_timeout()
            self._state_dispatch()
        except Exception as e:
            self.get_logger().error(f"状态机异常: {e}")
            self.stop_all()

    def _check_competition_timeout(self):
        """检查比赛是否超时，并定期日志剩余时间"""
        timeout = self.get_parameter("competition_timeout_sec").value
        elapsed = time.monotonic() - self.competition_start_time
        remaining = timeout - elapsed

        # 每 30s 日志剩余时间 + 当前进度
        if remaining > 0 and (self._last_time_log == 0 or
                              self._last_time_log - remaining >= 30.0):
            total = len(self.pickup_sequence)
            done = self.current_step_idx
            self.get_logger().info(
                f"[计时] 剩余 {remaining:.0f}s  "
                f"进度 {done}/{total}箱"
            )
            self._last_time_log = remaining

        if elapsed >= timeout:
            self.get_logger().warn(f"[计时] 比赛结束（{timeout:.0f}秒到）")
            if self.state in (VS.WAIT_PICK, VS.WAIT_PLACE):
                # 正在等待 → 跳过等待进入下一步
                if self.state == VS.WAIT_PICK:
                    self.send_auto_cmd(AUTO_CMD_PICK_DONE)
                else:
                    self.send_auto_cmd(AUTO_CMD_PLACE_DONE)
                self._transition_to(VS.NEXT_OR_FINISH)
            else:
                # 其它状态 → 直接结束
                self.send_velocity(0.0, 0.0)
                self.send_auto_cmd(AUTO_CMD_FINISH)
                self._transition_to(VS.IDLE)

    def _state_dispatch(self):
        s = self.state
        if s == VS.IDLE:
            pass  # 等待外部 start_task() 调用
        elif s == VS.SOLVE_TASK:
            self._run_solve_task()
        elif s == VS.NAV_BOX:
            self._run_nav_box()
        elif s == VS.WAIT_PICK:
            self._run_wait_pick()
        elif s == VS.NAV_ZONE:
            self._run_nav_zone()
        elif s == VS.WAIT_PLACE:
            self._run_wait_place()
        elif s == VS.NEXT_OR_FINISH:
            self._run_next_or_finish()
        elif s == VS.ERROR:
            pass

    # ==================== 各状态实现 ====================

    def start_task(self):
        """外部调用：启动任务"""
        if self.state != VS.IDLE:
            self.get_logger().warn(f"当前状态 {_STATE_NAMES[self.state]}，无法启动")
            return
        self.current_step_idx = 0
        self.pickup_sequence = []
        self.competition_start_time = time.monotonic()
        self._last_time_log = 0.0
        self._transition_to(VS.SOLVE_TASK)
        self.send_auto_cmd(AUTO_CMD_START, 0, 0)
        self.get_logger().info("=" * 40)
        self.get_logger().info("  任务启动！")
        self.get_logger().info(f"  总时长: {self.get_parameter('competition_timeout_sec').value:.0f}秒")
        self.get_logger().info("=" * 40)

    def estop(self):
        """急停"""
        self.stop_all()

    # --- SOLVE_TASK: 识别智力题（手册: 20秒内, 否则无高分区）---
    def _run_solve_task(self):
        mr = self._yolo_math_result
        if mr and mr.get("mod4") is not None:
            mod4 = int(mr["mod4"])
            self.high_score_zone = mod4
            self.pickup_sequence = self._generate_pickup_sequence(mod4)
            self.current_step_idx = 0
            self.get_logger().info(
                f"[智力题] mod4={mod4}  high_score_zone={self.high_score_zone}  "
                f"目标区类型={_BOX_TYPE_NAMES.get(FIELD_ZONES[self.field_id][mod4 % 4], '?')}"
            )
            self._transition_to(VS.NAV_BOX)
            return

        # 手册: 20秒内未能识别 → 本轮比赛没有高分区
        if self._elapsed() > self.math_timeout_sec:
            self.get_logger().warn(
                f"[智力题] {self.math_timeout_sec:.0f}秒超时，本轮无高分区"
            )
            self.high_score_zone = None
            self.pickup_sequence = self._generate_pickup_sequence(None)
            self.current_step_idx = 0
            self._transition_to(VS.NAV_BOX)

    # --- NAV_BOX: 导航到物资箱 ---
    def _run_nav_box(self):
        if self.current_step_idx >= len(self.pickup_sequence):
            self.get_logger().error(f"步骤 {self.current_step_idx} 超出序列")
            self.stop_all()
            return

        step = self.pickup_sequence[self.current_step_idx]
        box_type, zone_id, bx, by, byaw, zx, zy, zyaw = step

        # 首次进入时发送 Nav2 目标
        if self._elapsed() < 0.1:
            self.get_logger().info(
                f"[导航] 前往箱{self.current_step_idx+1}/{len(self.pickup_sequence)} "
                f"({_BOX_TYPE_NAMES.get(box_type,'?')}) "
                f"位置 ({bx:.2f}, {by:.2f})"
            )
            self.send_nav2_goal(bx, by, byaw)
            self.arrival.set_target(bx, by, byaw)
            return

        # 到达判断
        if self._nav_succeeded:
            self.get_logger().info(f"[到达] 到达物资箱 ({_BOX_TYPE_NAMES.get(box_type,'?')})")
            self.send_auto_cmd(AUTO_CMD_ARRIVED_BOX, self.current_step_idx, zone_id)
            self.send_velocity(0.0, 0.0)
            self._transition_to(VS.WAIT_PICK)

        # 超时保护
        if self._elapsed() > self.get_parameter("nav2_action_timeout_sec").value:
            self.get_logger().warn("[到达] 导航超时，强制到达")
            self.send_auto_cmd(AUTO_CMD_ARRIVED_BOX, self.current_step_idx, zone_id)
            self._transition_to(VS.WAIT_PICK)

    # --- WAIT_PICK: 检测立方体 → 转换坐标 → 发送机械臂目标 → 等待完成 ---
    def _run_wait_pick(self):
        # 阶段一: 等待 STM32 进入 PICK 状态 (ARRIVED_BOX settle 400ms)
        if self._elapsed() < 1.0:
            return

        # 阶段二: 等待 /detected_cube，使用 catch 模块检测 + 变换
        if not self._pick_done and _HAVE_CATCH:
            cube = self._latest_cube
            cube_age = time.monotonic() - self._cube_recv_time if cube else 999.0

            if cube is not None and cube_age < 2.0:
                cx, cy, cz = cube.pose.position.x, cube.pose.position.y, cube.pose.position.z

                # 雷达系坐标 → 缓存（catch 模块的 find_stable_points 在 arm 坐标域做稳定检测，
                # 但 vision_auto_task 在雷达域检测更直接，缓存原始雷达坐标）
                self._cube_cache.append((cx, cy, cz))
                if len(self._cube_cache) > CACHE_MAX_SIZE:
                    self._cube_cache.pop(0)

                # 使用 catch 模块的稳定检测函数
                indices, avg = find_stable_points(
                    self._cube_cache, STABLE_THRESHOLD_XY, STABLE_COUNT_REQUIRED)

                if indices is not None:
                    # 稳定 → 使用 catch 模块的坐标变换
                    avg_x, avg_y, avg_z = avg
                    arm_x, arm_y, arm_z, _ = transform_and_offset(avg_x, avg_y, avg_z)
                    arm_x += HALF_BOX_HEIGHT  # 中心→顶面

                    # 使用 catch 模块的工作空间检查
                    ok, dist = workspace_check(arm_x, arm_y, arm_z)
                    if ok:
                        self.send_arm_target(arm_x, arm_y, arm_z)
                        self._pick_done = True
                        self.get_logger().info(
                            f"[抓取] 坐标 ({arm_x:.3f}, {arm_y:.3f}, {arm_z:.3f})"
                        )
                    else:
                        self.get_logger().warn(f"[抓取] 坐标 {dist:.3f}m 超出工作空间")

        # 阶段三: 超时 → 发送 PICK_DONE
        if self._elapsed() > self.get_parameter("pick_timeout_sec").value:
            self.get_logger().info("[抓取] 超时，发送 PICK_DONE")
            self.send_auto_cmd(AUTO_CMD_PICK_DONE)
            self._cube_cache = []
            self._pick_done = False
            self._transition_to(VS.NAV_ZONE)

    # --- NAV_ZONE: 导航到归位区 ---
    def _run_nav_zone(self):
        if self.current_step_idx >= len(self.pickup_sequence):
            self.stop_all()
            return

        step = self.pickup_sequence[self.current_step_idx]
        box_type, zone_id, bx, by, byaw, zx, zy, zyaw = step

        if self._elapsed() < 0.1:
            self.get_logger().info(
                f"[导航] 前往归位区 {zone_id} "
                f"({_BOX_TYPE_NAMES.get(box_type,'?')}) "
                f"位置 ({zx:.2f}, {zy:.2f})"
            )
            self.send_nav2_goal(zx, zy, zyaw)
            self.arrival.set_target(zx, zy, zyaw)
            return

        if self._nav_succeeded:
            self.get_logger().info(f"[到达] Nav2 报告到达归位区 {zone_id}")
            self.send_auto_cmd(AUTO_CMD_ARRIVED_ZONE, self.current_step_idx, zone_id)
            self.send_velocity(0.0, 0.0)
            self._transition_to(VS.WAIT_PLACE)

        if self._elapsed() > self.get_parameter("nav2_action_timeout_sec").value:
            self.get_logger().warn("[到达] 导航超时，强制到达")
            self.send_auto_cmd(AUTO_CMD_ARRIVED_ZONE, self.current_step_idx, zone_id)
            self._transition_to(VS.WAIT_PLACE)

    # --- WAIT_PLACE: 发送放置坐标 → 等待放置完成 ---
    def _run_wait_place(self):
        # 阶段一: 发送地面放置坐标（转换为机械臂系）
        if self._elapsed() < 0.5 and not self._pick_done:
            if self.current_step_idx < len(self.pickup_sequence):
                step = self.pickup_sequence[self.current_step_idx]
                *_, zx, zy, _ = step
                # 地面放置: arm坐标 (x=向下够到地面, y=归位区侧向, z=归位区前向)
                place_x = -0.1   # 地面高度补偿（机械臂系 X 向下）
                place_y = 0.0    # 居中
                place_z = 0.3    # 前向伸到箱子位置
                self.send_arm_target(place_x, place_y, place_z)
                self._pick_done = True
                self.get_logger().info(f"[放置] 发送放置坐标 区位置({zx:.1f},{zy:.1f})")

        if self._elapsed() > self.get_parameter("place_timeout_sec").value:
            self.get_logger().info("[放置] 等待超时，发送 PLACE_DONE")
            self.send_auto_cmd(AUTO_CMD_PLACE_DONE)
            self._pick_done = False
            self._transition_to(VS.NEXT_OR_FINISH)

    # --- NEXT_OR_FINISH: 下一箱或结束 ---
    def _run_next_or_finish(self):
        self.current_step_idx += 1
        if self.current_step_idx < len(self.pickup_sequence):
            self.get_logger().info(
                f"[流程] 前往下一箱 ({self.current_step_idx + 1}/{len(self.pickup_sequence)})"
            )
            self.send_auto_cmd(AUTO_CMD_NEXT)
            self._transition_to(VS.NAV_BOX)
        else:
            self.get_logger().info("=" * 40)
            self.get_logger().info("  全部任务完成！")
            self.get_logger().info("=" * 40)
            self.send_auto_cmd(AUTO_CMD_FINISH)
            self._transition_to(VS.IDLE)


# ================================================================
# 入口
# ================================================================
# ================================================================
# 一键启动 —— 拉起所有 ROS 节点 + 本状态机
# ================================================================

def _prompt_config():
    """赛前配置交互"""
    print()
    print("╔══════════════════════════════════════════════╗")
    print("║         赛前配置                             ║")
    print("╚══════════════════════════════════════════════╝")
    print(" 1=食品  2=工具  3=仪器  4=药品")
    print()

    field_id = None
    while field_id not in (1, 2):
        try:
            field_id = int(input(" 场地号 (1 或 2): ").strip())
        except (ValueError, EOFError):
            pass

    print(f" 使用场地 {field_id}")
    print(" 内排顺序（靠归位区那边，4 个数字空格分隔）:")

    inner = []
    while len(inner) != 4 or sorted(inner) != [1, 2, 3, 4]:
        try:
            raw = input("   例: 2 1 4 3  → ").strip().split()
            inner = [int(x) for x in raw]
            if len(inner) != 4 or sorted(inner) != [1, 2, 3, 4]:
                print("   错误: 需要 1/2/3/4 各一个，重新输入")
                inner = []
        except (ValueError, EOFError):
            inner = []

    # 写入配置
    FIELD_BOXES[field_id][1] = inner
    print(f" 内排: {inner}")
    print()

    return field_id


def main(args=None):
    # === 赛前配置 ===
    field_id = _prompt_config()

    # === 项目路径 ===
    fastlio_dir = str(_PROJECT / "fastlio2_v2")
    nav2_dir = str(_PROJECT / "nav2_ws1")
    map_dir = _PROJECT / "map"
    pcd_map = str(map_dir / "map.pcd")
    pgm_map = str(map_dir / "map.yaml")

    ros_setup = "source /opt/ros/jazzy/setup.bash"

    print("╔══════════════════════════════════════════════╗")
    print("║  启动比赛全流程 ROS 节点                      ║")
    print("╚══════════════════════════════════════════════╝")

    # LiDAR 驱动
    launch(
        f"cd {fastlio_dir} && {ros_setup} && source install/setup.bash && "
        f"ros2 launch unitree_lidar_ros2 launch.py",
        name="LiDAR",
    )

    # ICP 定位 (FAST-LIO2 + transform_fusion)
    launch(
        f"cd {fastlio_dir} && {ros_setup} && source install/setup.bash && "
        f"export AMENT_PREFIX_PATH=\"$PWD/install/fast_lio_localization:$AMENT_PREFIX_PATH\" && "
        f"export PYTHONPATH=\"$PYTHONPATH:$HOME/.local/lib/python3.12/site-packages\" && "
        f"ros2 launch fast_lio_localization 1.launch.py "
        f"  map:={pcd_map} config_file:=unilidar_l2.yaml rviz:=true "
        f"  map_voxel_size:=0.01 scan_voxel_size:=0.03 "
        f"  freq_localization:=2.0 localization_threshold:=0.9",
        name="ICP",
    )

    # odometry→TF 桥
    odom_bin = f"{fastlio_dir}/build/fast_lio/odometry_to_tf"
    if os.path.isfile(odom_bin):
        launch(f"cd {fastlio_dir} && {ros_setup} && source install/setup.bash && {odom_bin}", name="TF桥")
    else:
        print(f"  [TF桥] 未找到 {odom_bin}，跳过")

    # Nav2
    launch(
        f"cd {nav2_dir} && {ros_setup} && source install/setup.bash && "
        f"ros2 launch dog_nav2_bringup nav2_fastlio_static_map.launch.py "
        f"  map:={pgm_map}",
        name="Nav2",
    )

    # 串口桥
    launch(
        f"cd {nav2_dir} && {ros_setup} && source install/setup.bash && "
        f"ros2 launch dog_nav2_bringup chassis_serial_bridge.launch.py "
        f"  serial_port:=/dev/ttyACM0 baud_rate:=115200 "
        f"  cmd_vel_topic:=/cmd_vel send_rate_hz:=50.0 "
        f"  active_state:=1 idle_state:=0",
        name="串口桥",
    )

    # 注册清理
    atexit.register(cleanup_all)
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            signal.signal(sig, lambda *_: (cleanup_all(), sys.exit(0)))
        except Exception:
            pass

    print("  ⏳ 等待节点启动（8 秒）...")
    time.sleep(8)
    print("  ➤ 自动初始化 ICP 定位（原点, X正向）...")

    # 发布 /initialpose 自动初始化 ICP（替代手动点击 RViz）
    init_pose_script = _PROJECT / "py" / "control" / "init_pose.py"
    subprocess.run(
        ["bash", "-c",
         f"source /opt/ros/jazzy/setup.bash && "
         f"RMW_IMPLEMENTATION=rmw_cyclonedds_cpp python3 {init_pose_script}"],
    )
    time.sleep(1)  # 等 ICP 处理完初始位姿
    print("  ✅ ICP 初始化完成（如需重新定位，在 RViz 中点 2D Pose Estimate）")
    print()

    # === 启动本节点 ===
    rclpy.init(args=args)

    # 用输入 field_id 覆盖默认参数
    node = VisionAutoTaskNode()
    node.field_id = field_id
    # 清除旧序列
    node.pickup_sequence = []
    node.zone_sequence = []

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    # 可选：添加 cube_detector 作为模块节点（否则退化到等待外部 topic）
    if _HAVE_CUBE_DETECTOR and _CubeDetector is not None:
        try:
            cube_node = _CubeDetector()
            executor.add_node(cube_node)
            print("  [CUBE] cube_detector 已作为模块加载")
        except Exception as e:
            print(f"  [CUBE] 模块加载失败: {e}，需手动启动 cube_detector.py")

    # 提供 CLI 交互
    print("\nVisionAutoTaskNode 已启动")
    print("  start → 开始任务")
    print("  estop → 急停")
    print("  quit  → 退出（同时关闭所有节点）\n")

    import threading
    def cli_thread():
        while rclpy.ok():
            try:
                cmd = input().strip().lower()
                if cmd == "start":
                    node.start_task()
                elif cmd == "estop":
                    node.estop()
                elif cmd == "quit":
                    rclpy.shutdown()
                    break
            except (EOFError, KeyboardInterrupt):
                break

    t = threading.Thread(target=cli_thread, daemon=True)
    t.start()

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cleanup_all()


if __name__ == "__main__":
    main()
