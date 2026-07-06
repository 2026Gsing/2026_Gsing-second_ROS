#!/usr/bin/env python3
"""
vision_auto_task_node.py — 视觉全自动任务执行节点

职责：
  1. 视觉状态机 IDLE → SOLVE_TASK → FIND_BOX → NAV_BOX → WAIT_PICK
                        → NAV_ZONE → WAIT_PLACE → NEXT_OR_FINISH
  2. YOLO 物资箱检测 + 数学题识别（可选，可通过文件或 ROS topic 输入）
  3. 发送 Nav2 导航目标
  4. 到达检测（订阅 /localization）
  5. 发布自动任务命令到 /vision/auto_cmd（由 cmd_vel_chassis_serial.py 转发 0x15）
  6. 发布精细控速到 /vision_cmd_vel

使用方式：
  python3 py/vision_auto_task_node.py

依赖：
  rclpy, nav2_msgs, geometry_msgs, nav_msgs, std_msgs, pyyaml
"""

import math
import sys
import time
import json
from pathlib import Path

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
_HERE = Path(__file__).resolve().parent
_PROJECT = _HERE.parent
_VISION_DIR = _PROJECT / "vision"            # YOLO 代码整合目录
_YOLO_SRC = _VISION_DIR / "src"
_VISION_WEIGHTS = _VISION_DIR / "weights"
_VISION_CONFIG = _VISION_DIR / "config"
_DEFAULT_WAYPOINTS = _HERE / "config" / "competition_poses.yaml"


class VisionAutoTaskNode(Node):
    def __init__(self):
        super().__init__("vision_auto_task")

        # ======================== 参数 ========================
        self.declare_parameter("waypoints_file", str(_DEFAULT_WAYPOINTS))
        self.declare_parameter("arrival_pos_threshold", 0.25)   # m
        self.declare_parameter("arrival_angle_threshold", 0.30) # rad
        self.declare_parameter("arrival_settle_frames", 5)
        self.declare_parameter("pick_timeout_sec", 5.0)         # 抓取等待超时
        self.declare_parameter("place_timeout_sec", 5.0)        # 放置等待超时
        self.declare_parameter("total_boxes", 4)
        self.declare_parameter("yolo_decision_file", str(_VISION_CONFIG / "decision_state.json"))
        self.declare_parameter("nav2_action_timeout_sec", 30.0)

        # ======================== 状态机 ========================
        self.state = VS.IDLE
        self.current_box_idx = 0     # 当前正在处理的箱号 (0-based)
        self.total_boxes = self.get_parameter("total_boxes").value
        self.zone_sequence = []      # 由数学题结果决定的归位区序列
        self.target_class = None     # 当前要抓的物资类别
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

        # ======================== 发布 ========================
        # auto_cmd_payload: JSON {"cmd":2,"target":0,"zone":1} 发给 serial 桥
        self.auto_cmd_pub = self.create_publisher(String, "/vision/auto_cmd", 10)
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

    def _elapsed(self) -> float:
        return time.monotonic() - self._state_enter_time

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
            self._state_dispatch()
        except Exception as e:
            self.get_logger().error(f"状态机异常: {e}")
            self.stop_all()

    def _state_dispatch(self):
        s = self.state
        if s == VS.IDLE:
            pass  # 等待外部 start_task() 调用
        elif s == VS.SOLVE_TASK:
            self._run_solve_task()
        elif s == VS.FIND_BOX:
            self._run_find_box()
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
        self.current_box_idx = 0
        self._transition_to(VS.SOLVE_TASK)
        self.send_auto_cmd(AUTO_CMD_START, 0, 0)
        self.get_logger().info("=" * 40)
        self.get_logger().info("  任务启动！")
        self.get_logger().info("=" * 40)

    def estop(self):
        """急停"""
        self.stop_all()

    # --- SOLVE_TASK: 识别智力题 ---
    def _run_solve_task(self):
        mr = self._yolo_math_result
        if mr and mr.get("mod4") is not None:
            mod4 = int(mr["mod4"])
            # mod4 → zone 分配逻辑（根据赛事规则调整）
            # 示例: mod4=0 → [0,1,2,3]; mod4=1 → [1,2,3,0]; ...
            self.zone_sequence = [(mod4 + i) % self.total_boxes for i in range(self.total_boxes)]
            self.target_class = mr.get("target_class", "tool")
            self.get_logger().info(
                f"[智力题] mod4={mod4}  zone_sequence={self.zone_sequence}  "
                f"target_class={self.target_class}"
            )
            self._transition_to(VS.FIND_BOX)
            return

        # 超时处理（防止卡死）
        if self._elapsed() > 60.0:
            self.get_logger().warn("[智力题] 超时，使用默认序列")
            self.zone_sequence = list(range(self.total_boxes))
            self.target_class = "tool"
            self._transition_to(VS.FIND_BOX)

    # --- FIND_BOX: 识别物资箱 ---
    def _run_find_box(self):
        det = self._yolo_detections
        if det and "slot_id" in str(det):
            self.get_logger().info(f"[检测] 识别到目标: {det}")
            self._transition_to(VS.NAV_BOX)
            return

        if self._elapsed() > 30.0:
            self.get_logger().warn("[检测] 超时，使用默认箱号")
            self._transition_to(VS.NAV_BOX)

    # --- NAV_BOX: 导航到物资箱 ---
    def _run_nav_box(self):
        # 首次进入时发送 Nav2 目标
        if self._elapsed() < 0.1:
            boxes = self.waypoints.get("boxes", [])
            if self.current_box_idx < len(boxes):
                wp = boxes[self.current_box_idx]
                self.send_nav2_goal(wp["x"], wp["y"], wp["yaw"])
                self.arrival.set_target(wp["x"], wp["y"], wp["yaw"])
            else:
                self.get_logger().error(f"箱号 {self.current_box_idx} 超出配置")
                self.stop_all()
            return

        # 到达判断
        if self._nav_succeeded:
            self.get_logger().info("[到达] Nav2 报告到达物资箱")
            zone_id = self.zone_sequence[self.current_box_idx] if self.current_box_idx < len(self.zone_sequence) else 0
            self.send_auto_cmd(AUTO_CMD_ARRIVED_BOX, self.current_box_idx, zone_id)
            self.send_velocity(0.0, 0.0)
            self._transition_to(VS.WAIT_PICK)

        # 超时保护
        if self._elapsed() > self.get_parameter("nav2_action_timeout_sec").value:
            self.get_logger().warn("[到达] 导航超时，强制到达")
            zone_id = self.zone_sequence[self.current_box_idx] if self.current_box_idx < len(self.zone_sequence) else 0
            self.send_auto_cmd(AUTO_CMD_ARRIVED_BOX, self.current_box_idx, zone_id)
            self._transition_to(VS.WAIT_PICK)

    # --- WAIT_PICK: 等待抓取完成 ---
    def _run_wait_pick(self):
        if self._elapsed() > self.get_parameter("pick_timeout_sec").value:
            self.get_logger().info("[抓取] 等待超时，发送 PICK_DONE")
            self.send_auto_cmd(AUTO_CMD_PICK_DONE)
            self._transition_to(VS.NAV_ZONE)

    # --- NAV_ZONE: 导航到归位区 ---
    def _run_nav_zone(self):
        if self._elapsed() < 0.1:
            zone_idx = self.zone_sequence[self.current_box_idx] if self.current_box_idx < len(self.zone_sequence) else 0
            zones = self.waypoints.get("zones", [])
            zone_wp = next((z for z in zones if z["id"] == zone_idx), zones[0] if zones else None)
            if zone_wp:
                self.send_nav2_goal(zone_wp["x"], zone_wp["y"], zone_wp["yaw"])
                self.arrival.set_target(zone_wp["x"], zone_wp["y"], zone_wp["yaw"])
                self.get_logger().info(f"[导航] 前往归位区 {zone_idx}")
            else:
                self.get_logger().error(f"归位区 {zone_idx} 未在配置中找到")
                self.stop_all()
            return

        if self._nav_succeeded:
            zone_idx = self.zone_sequence[self.current_box_idx] if self.current_box_idx < len(self.zone_sequence) else 0
            self.get_logger().info(f"[到达] Nav2 报告到达归位区 {zone_idx}")
            self.send_auto_cmd(AUTO_CMD_ARRIVED_ZONE, self.current_box_idx, zone_idx)
            self.send_velocity(0.0, 0.0)
            self._transition_to(VS.WAIT_PLACE)

        if self._elapsed() > self.get_parameter("nav2_action_timeout_sec").value:
            self.get_logger().warn("[到达] 导航超时，强制到达")
            zone_idx = self.zone_sequence[self.current_box_idx] if self.current_box_idx < len(self.zone_sequence) else 0
            self.send_auto_cmd(AUTO_CMD_ARRIVED_ZONE, self.current_box_idx, zone_idx)
            self._transition_to(VS.WAIT_PLACE)

    # --- WAIT_PLACE: 等待放置完成 ---
    def _run_wait_place(self):
        if self._elapsed() > self.get_parameter("place_timeout_sec").value:
            self.get_logger().info("[放置] 等待超时，发送 PLACE_DONE")
            self.send_auto_cmd(AUTO_CMD_PLACE_DONE)
            self._transition_to(VS.NEXT_OR_FINISH)

    # --- NEXT_OR_FINISH: 下一箱或结束 ---
    def _run_next_or_finish(self):
        self.current_box_idx += 1
        if self.current_box_idx < self.total_boxes:
            self.get_logger().info(f"[流程] 前往下一箱 ({self.current_box_idx + 1}/{self.total_boxes})")
            self.send_auto_cmd(AUTO_CMD_NEXT)
            self._transition_to(VS.FIND_BOX)
        else:
            self.get_logger().info("=" * 40)
            self.get_logger().info("  全部任务完成！")
            self.get_logger().info("=" * 40)
            self.send_auto_cmd(AUTO_CMD_FINISH)
            self._transition_to(VS.IDLE)


# ================================================================
# 入口
# ================================================================
def main(args=None):
    rclpy.init(args=args)
    node = VisionAutoTaskNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    # 提供 CLI 交互
    print("\nVisionAutoTaskNode 已启动")
    print("  命令: start → 开始任务")
    print("        estop → 急停")
    print("        quit  → 退出\n")

    # 非阻塞输入监听
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


if __name__ == "__main__":
    main()
