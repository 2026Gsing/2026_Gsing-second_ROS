#!/usr/bin/env python3
"""
auto_task.py — 全场自动任务（Nav2 导航版）

流程（基于 Nav2 导航，模仿 box_pick_node.py 的到达检测机制）：
  1. 启动前置节点（LiDAR/ICP/Nav2/串口桥）
  2. Nav2 导航到拾取点 (1.4, 0, 0)
  3. 到达后自动检测 → 抓取箱子（cube_detector + catch）
  4. Nav2 导航到放置点 (4, 0, 0)
  5. 到达后放下箱子（根据之前拾取的位置确定放置参数）
  6. 完成

使用方式：
./ros-run.sh py/control/auto_task.py

运行时调参可通过 ros2 param（详见 __init__ 中的 declare_parameter）
"""

import json
import math
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from visualization_msgs.msg import Marker
from std_msgs.msg import String

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from launch_utils import start_prerequisites, cleanup_all
from catch import transform_and_offset, validate_arm_target, stm32_will_accept
from arrival_detector import quaternion_to_yaw

_HERE = Path(__file__).resolve().parent          # py/control/

# ════════════════════════════════════════════════════════════════
# 串口协议常量
# ════════════════════════════════════════════════════════════════
AUTO_CMD_START = 1
AUTO_CMD_ARRIVED_BOX = 2
AUTO_CMD_ARRIVED_ZONE = 4
AUTO_CMD_FINISH = 7

ARM_EVENT_PICK_DONE = 1
ARM_EVENT_PLACE_DONE = 2

ARM_MISSION_PICK_TO_BACK = 1
ARM_MISSION_BACK_TO_PLACE = 2
ARM_MISSION_HAS_PICK = 0x01
ARM_MISSION_HAS_BACK = 0x02
ARM_MISSION_HAS_PLACE = 0x04

# ════════════════════════════════════════════════════════════════
# 全局可调参数（直接改这里，也可以通过 ros2 param set）
# ════════════════════════════════════════════════════════════════
PICKUP_X = 1.4
PICKUP_Y = 0.0
PICKUP_YAW = 0.0  # 朝向（弧度）

DROP_X = 4.0
DROP_Y = 0.0
DROP_YAW = 0.0

PICK_TIMEOUT = 30.0      # 抓取超时 (s)
PLACE_TIMEOUT = 25.0     # 放置超时 (s)
DETECT_TIMEOUT = 15.0    # 检测等待超时 (s)
NAV_TIMEOUT = 120.0      # 单段导航超时 (s)

ARRIVAL_DIST_THRESHOLD = 0.15   # 距离目标多近算到达 (m)


class AutoTask(Node):
    def __init__(self):
        super().__init__("auto_task")

        # ============ 线程锁 ============
        self._lock = threading.RLock()

        # ============ 子进程 ============
        self._cube_proc = None
        self._catch_proc = None

        # ============ 定位 / 检测数据 ============
        self._latest_localization = None   # Odometry (map 系)
        self._latest_cube = None           # Marker (lidar 系)
        self._cube_received_time = 0.0

        # ============ Nav2 导航 ============
        self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._nav_goal_handle = None
        self._nav_succeeded = False

        # ============ 自动到达检测 ============
        self._nav_goal_pos = None          # 当前 Nav2 目标 (x, y)
        self._arrival_triggered = False

        # ============ 阶段同步 ============
        # 每个阶段（导航→到达→操作→完成）用事件来同步主线程与工作线程
        self._phase_event = threading.Event()
        self._phase_event.set()            # 初始无障碍
        self._current_phase = "IDLE"       # IDLE / NAV_TO_PICK / PICKING / NAV_TO_DROP / PLACING

        # ============ 拾取信息（记录箱子坐标供放置参考） ============
        self._picked_box_map_pos = None    # (x, y) map 系拾取坐标

        # ============ 发布器 ============
        self.pub_cmd_vel = self.create_publisher(Twist, "/vision_cmd_vel", 10)
        self.pub_auto_cmd = self.create_publisher(String, "/vision/auto_cmd", 10)
        self.pub_arm_mission = self.create_publisher(String, "/vision/arm_mission", 10)
        self._pub_initialpose = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 1)

        # ============ 订阅器 ============
        self.create_subscription(Marker, "/detected_cube", self._cube_cb, 10)
        self.create_subscription(Odometry, "/localization", self._localization_cb, 10)

        self.get_logger().info("AutoTask 已启动")

    # ──────────────────────────────────────────────────────────
    # 回调
    # ──────────────────────────────────────────────────────────

    def _cube_cb(self, msg):
        with self._lock:
            self._latest_cube = msg
            self._cube_received_time = time.monotonic()

    def _localization_cb(self, msg):
        with self._lock:
            self._latest_localization = msg

            # 自动到达检测（仅当有导航目标且未触发时才检测）
            if self._arrival_triggered:
                return
            if self._nav_goal_pos is None:
                return

            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            gx, gy = self._nav_goal_pos
            dist = math.hypot(x - gx, y - gy)

            if dist < ARRIVAL_DIST_THRESHOLD:
                self.get_logger().info(
                    f"[自动到达] 目标 ({gx:.2f}, {gy:.2f})  "
                    f"当前 ({x:.3f}, {y:.3f}) 距离={dist:.3f}m"
                )
                self._arrival_triggered = True
                self._nav_goal_pos = None
                self._on_arrived()

    # ──────────────────────────────────────────────────────────
    # Nav2 导航回调
    # ──────────────────────────────────────────────────────────

    def _nav_goal_response_cb(self, future):
        goal_handle = future.result()
        with self._lock:
            if not goal_handle.accepted:
                self.get_logger().warn("[NAV] 目标被拒绝")
                self._nav_goal_handle = None
                return
            self.get_logger().info("[NAV] 目标已接受")
            self._nav_goal_handle = goal_handle
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self._nav_result_cb)

    def _nav_result_cb(self, future):
        result = future.result()
        with self._lock:
            if result and result.status == 4:  # SUCCEEDED
                self._nav_succeeded = True
                self.get_logger().info("[NAV] ✅ 导航成功")
                # 直接触发到达处理，不依赖 ICP 到达检测
                # （ICP 位置与 Nav2 目标可能偏差 0.3-0.5m，导致自动到达检测永不触发）
                self._on_arrived()
            else:
                status_names = {
                    0: "UNKNOWN", 1: "ACCEPTED", 2: "EXECUTING",
                    3: "CANCELED", 4: "SUCCEEDED", 5: "FAILED",
                }
                s = status_names.get(result.status, f"???({result.status})")
                self.get_logger().warn(f"[NAV] ❌ 导航结束: {s}")
            self._nav_goal_handle = None

    def _send_nav_goal(self, x, y, yaw=0.0):
        """发送 Nav2 导航目标（异步）"""
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        if not self._nav_client.wait_for_server(timeout_sec=8.0):
            self.get_logger().error("[NAV] Nav2 action server 不可用（等待 8s 超时）")
            return False

        with self._lock:
            self._nav_succeeded = False
            self._nav_goal_pos = (x, y)
            self._arrival_triggered = False

        send_future = self._nav_client.send_goal_async(goal)
        send_future.add_done_callback(self._nav_goal_response_cb)
        self.get_logger().info(f"[NAV] 发送目标: ({x:.2f}, {y:.2f}, yaw={yaw:.2f})")
        return True

    # ──────────────────────────────────────────────────────────
    # 串口指令发送
    # ──────────────────────────────────────────────────────────

    def send_vel(self, vx, wz):
        """通过 /vision_cmd_vel 发送速度指令（优先级高于 Nav2 的 /cmd_vel）"""
        msg = Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(wz)
        self.pub_cmd_vel.publish(msg)

    def send_auto_cmd(self, cmd, target=0, zone=0):
        data = json.dumps({"cmd": cmd, "target": target, "zone": zone})
        self.pub_auto_cmd.publish(String(data=data))
        self.get_logger().info(f"  AUTO_CMD: cmd={cmd} target={target} zone={zone}")

    def send_arm_mission(self, mode, flags, sequences):
        payload = {"mode": mode, "flags": flags}
        for key in ("pick", "back", "place"):
            if key in sequences:
                payload[key] = sequences[key]
        self.pub_arm_mission.publish(String(data=json.dumps(payload)))
        self.get_logger().info(f"  ARM_MISSION: mode={mode} flags={flags}")

    # ──────────────────────────────────────────────────────────
    # 到达处理（由 _localization_cb 中的自动检测触发）
    # ──────────────────────────────────────────────────────────

    def _on_arrived(self):
        """
        到达目标点后的处理。
        根据当前阶段判断是执行拾取还是放置。
        """
        with self._lock:
            phase = self._current_phase

        if phase == "NAV_TO_PICK":
            self.get_logger().info("=" * 50)
            self.get_logger().info("[→] 到达拾取点，启动检测流程")
            self.get_logger().info("=" * 50)
            self._set_phase("PICKING")
            # 先发停止指令（覆盖 Nav2）
            self.send_vel(0.0, 0.0)
            # 在独立线程中执行检测+抓取（不阻塞 spin）
            t = threading.Thread(target=self._do_pick_flow, daemon=True)
            t.start()

        elif phase == "NAV_TO_DROP":
            self.get_logger().info("=" * 50)
            self.get_logger().info("[→] 到达放置点，启动放置流程")
            self.get_logger().info("=" * 50)
            self._set_phase("PLACING")
            self.send_vel(0.0, 0.0)
            t = threading.Thread(target=self._do_place_flow, daemon=True)
            t.start()

    # ──────────────────────────────────────────────────────────
    # 拾取流程
    # ──────────────────────────────────────────────────────────

    def _do_pick_flow(self):
        """检测立方体 → 判断可达性 → 抓取"""
        try:
            # ── 启动 cube_detector ──
            self._start_cube_detector()

            # ── 等待检测结果 ──
            self.get_logger().info(f"[检测] 等待 cube_detector 数据（最多 {DETECT_TIMEOUT:.0f}s）...")
            deadline = time.monotonic() + DETECT_TIMEOUT
            has_cube = False
            while time.monotonic() < deadline:
                time.sleep(0.2)
                with self._lock:
                    if (self._latest_cube is not None
                            and time.monotonic() - self._cube_received_time < 3.0):
                        has_cube = True
                        cube = self._latest_cube
                        break
            if not has_cube:
                self.get_logger().warn(f"[检测] ⏰ 超时: 未检测到立方体")
                self._finish_phase(phase="NAV_TO_DROP")
                return

            cx, cy, cz = cube.pose.position.x, cube.pose.position.y, cube.pose.position.z
            self.get_logger().info(f"[原始] 雷达系: x={cx:.3f}  y={cy:.3f}  z={cz:.3f}")

            # ── 变换到机械臂系 ──
            arm_x, arm_y, arm_z, _ = transform_and_offset(cx, cy, cz)
            arm_x += 0.125  # HALF_BOX_HEIGHT
            self.get_logger().info(f"[变换] 机械臂系(补偿后): x={arm_x:.3f}  y={arm_y:.3f}  z={arm_z:.3f}")

            # ── 可达性判断 ──
            reachable, reason = validate_arm_target(arm_x, arm_y, arm_z)
            if reachable:
                stm32_ok, stm32_reason = stm32_will_accept(arm_x, arm_y, arm_z)
                if not stm32_ok:
                    reachable = False
                    reason = f"STM32拒绝({stm32_reason})"

            if not reachable:
                self.get_logger().warn(f"[检测] 不可达({reason})，跳过抓取")
                self._finish_phase(phase="NAV_TO_DROP")
                return

            self.get_logger().info("[检测] ✓ 可达，启动抓取")

            # ── 记录箱子在 map 系下的坐标（供放置参考） ──
            loc = self._latest_localization
            if loc:
                robot = loc.pose.pose
                yaw = quaternion_to_yaw(robot.orientation)
                rx = robot.position.x
                ry = robot.position.y
                mx = rx + cx * math.cos(yaw) - cy * math.sin(yaw)
                my = ry + cx * math.sin(yaw) + cy * math.cos(yaw)
                with self._lock:
                    self._picked_box_map_pos = (mx, my)
                self.get_logger().info(f"[记录] 箱子 map 坐标: ({mx:.3f}, {my:.3f})")

            # ── 启动 catch.py 执行抓取 ──
            # catch.py 内部会处理：稳定检测 → 推进 STM32 状态机 → 发坐标 → 等待完成
            self._start_catch()
            if self._catch_proc is not None:
                self.get_logger().info(f"[抓取] 等待 catch.py (PID={self._catch_proc.pid})...")
                try:
                    self._catch_proc.wait(timeout=PICK_TIMEOUT)
                    self.get_logger().info("[抓取] ✅ catch.py 正常退出")
                except subprocess.TimeoutExpired:
                    self.get_logger().warn(f"[抓取] ⏰ catch.py 超时 ({PICK_TIMEOUT:.0f}s)")
                    try:
                        self._catch_proc.kill()
                        self._catch_proc.wait(timeout=3.0)
                    except Exception:
                        pass
            else:
                # catch.py 启动失败，降级：手动推进状态机 + 发 arm_mission
                self.get_logger().warn("[抓取] catch.py 启动失败，降级发送")
                self.send_auto_cmd(AUTO_CMD_START)
                time.sleep(0.5)
                self.send_auto_cmd(AUTO_CMD_ARRIVED_BOX, target=1)
                time.sleep(0.5)
                self.send_arm_mission(
                    mode=ARM_MISSION_PICK_TO_BACK,
                    flags=ARM_MISSION_HAS_PICK | ARM_MISSION_HAS_BACK | 0x08,
                    sequences={
                        "pick": {"x": -0.21, "y": 0.25, "z": -0.4},
                        "back": {"x": -0.25, "y": 0.0, "z": -0.35},
                        "back_side": 1,
                    },
                )
                self.get_logger().info(f"[抓取] 等待抓取完成（{PICK_TIMEOUT:.0f}s）...")
                time.sleep(PICK_TIMEOUT)

            self.get_logger().info("[抓取] ✅ 拾取阶段完成")
            self._finish_phase(phase="NAV_TO_DROP")

        except Exception as e:
            self.get_logger().error(f"[抓取异常] {e}")
            self._finish_phase(phase="NAV_TO_DROP")

    # ──────────────────────────────────────────────────────────
    # 放置流程
    # ──────────────────────────────────────────────────────────

    def _do_place_flow(self):
        """到达放置点后执行放置"""
        try:
            self.get_logger().info("[放置] 发送放置指令...")

            # 根据之前拾取坐标决定放置参数
            with self._lock:
                picked = self._picked_box_map_pos

            if picked:
                # 根据箱子 x 坐标确定归位区 ID（例如 x<2 → zone 0, x>=2 → zone 1）
                zone_id = 0 if picked[0] < 2.0 else 1
                self.get_logger().info(
                    f"[放置] 箱子拾取于 ({picked[0]:.2f}, {picked[1]:.2f}) → 归位区 {zone_id}"
                )
            else:
                zone_id = 0
                self.get_logger().info("[放置] 无拾取坐标记录，默认归位区 0")

            self.send_auto_cmd(AUTO_CMD_ARRIVED_ZONE, target=1, zone=zone_id)
            time.sleep(0.5)

            self.send_arm_mission(
                mode=ARM_MISSION_BACK_TO_PLACE,
                flags=ARM_MISSION_HAS_PLACE,
                sequences={
                    "place": {"x": -0.25, "y": 0.0, "z": -0.35},
                },
            )

            # 等待放置（用 sleep 等待 STM32 执行，也可扩展为监听 arm_event）
            self.get_logger().info(f"[放置] 等待放置完成（超时 {PLACE_TIMEOUT:.0f}s）...")
            time.sleep(PLACE_TIMEOUT)

            self.get_logger().info("[放置] ✅ 放置完成")
            self._finish_phase(phase="DONE")

        except Exception as e:
            self.get_logger().error(f"[放置异常] {e}")
            self._finish_phase(phase="DONE")

    # ──────────────────────────────────────────────────────────
    # 阶段管理
    # ──────────────────────────────────────────────────────────

    def _set_phase(self, phase):
        with self._lock:
            self._current_phase = phase

    def _finish_phase(self, phase):
        """
        当前阶段操作完成。
        - 清理子进程
        - 重置状态
        - 设置下一阶段
        - 通知主线程
        """
        self._cleanup_subprocesses()
        with self._lock:
            self._current_phase = phase
            self._arrival_triggered = False
            self._latest_cube = None
        self._phase_event.set()
        self.get_logger().info(f"[阶段] → {phase}")

    def _cleanup_subprocesses(self):
        """停止 cube_detector 和 catch 子进程"""
        for attr in ("_cube_proc", "_catch_proc"):
            proc = getattr(self, attr, None)
            if proc is None:
                continue
            try:
                proc.send_signal(signal.SIGINT)
                proc.wait(timeout=3.0)
            except Exception:
                try:
                    proc.kill()
                    proc.wait()
                except Exception:
                    pass
            setattr(self, attr, None)

    def start_phase(self, phase_name, nav_x, nav_y, nav_yaw=0.0):
        """
        启动一个新阶段：清除同步事件 → 设置阶段 → 发送 Nav2 目标。
        导航到达后的操作由 _on_arrived 根据 _current_phase 触发。
        """
        self._phase_event.clear()
        self._set_phase(phase_name)
        ok = self._send_nav_goal(nav_x, nav_y, nav_yaw)
        if not ok:
            self.get_logger().error(f"[阶段] Nav2 action server 不可用，阶段 {phase_name} 无法开始")
            self._phase_event.set()  # 让 wait_phase 立即返回，不自旋等超时

    def wait_phase(self, timeout=180.0):
        """
        等待当前阶段完成。
        在循环中持续 spin 处理回调（定位、导航到达检测等）。
        返回是否正常完成（非超时）。
        """
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._phase_event.is_set():
                return True
        if rclpy.ok():
            self.get_logger().warn(f"[阶段] ⏰ 阶段超时 ({timeout:.0f}s)")
        return False

    # ──────────────────────────────────────────────────────────
    # 子进程管理
    # ──────────────────────────────────────────────────────────

    def _start_cube_detector(self):
        if self._cube_proc is not None:
            self.get_logger().info("[CUBE] 已在运行，跳过")
            return
        script = str(_HERE / "cube_detector.py")
        logdir = _HERE.parent.parent / "logs" / "vision"
        logdir.mkdir(parents=True, exist_ok=True)
        logfile = str(logdir / f"{time.strftime('%Y-%m-%d_%H%M%S')}_cube_detector.log")
        try:
            f = open(logfile, "w")
            self._cube_proc = subprocess.Popen(
                [sys.executable, script],
                stdout=f, stderr=subprocess.STDOUT,
            )
            f.close()
            self.get_logger().info(f"[CUBE] PID={self._cube_proc.pid} → {logfile}")
        except Exception as e:
            self.get_logger().error(f"[CUBE] 启动失败: {e}")

    def _start_catch(self):
        if self._catch_proc is not None:
            self.get_logger().info("[CATCH] 已在运行，跳过")
            return
        script = str(_HERE / "catch.py")
        logdir = _HERE.parent.parent / "logs" / "arm"
        logdir.mkdir(parents=True, exist_ok=True)
        logfile = str(logdir / f"{time.strftime('%Y-%m-%d_%H%M%S')}_catch.log")
        try:
            f = open(logfile, "w")
            self._catch_proc = subprocess.Popen(
                [sys.executable, script],
                stdout=f, stderr=subprocess.STDOUT,
            )
            f.close()
            self.get_logger().info(f"[CATCH] PID={self._catch_proc.pid} → {logfile}")
        except Exception as e:
            self.get_logger().error(f"[CATCH] 启动失败: {e}")

    # ──────────────────────────────────────────────────────────
    # 发布 /initialpose
    # ──────────────────────────────────────────────────────────

    def publish_initialpose(self, x=0.0, y=0.0):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.w = 1.0
        self._pub_initialpose.publish(msg)
        self.get_logger().info(f"[启动] /initialpose ({x}, {y})")


# ════════════════════════════════════════════════════════════════
# 入口
# ════════════════════════════════════════════════════════════════

def _sigint_handler(sig, frame):
    print("\n[退出] 收到 Ctrl+C，正在关闭...")
    cleanup_all()
    rclpy.shutdown()


def main():
    signal.signal(signal.SIGINT, _sigint_handler)
    os.environ.setdefault("ROS_LOG_DIR", str(_HERE.parent.parent / "logs" / "ros"))

    rclpy.init()
    print("╔══════════════════════════════════════════════╗")
    print("║  自动任务（Nav2 导航版）                    ║")
    print("║  流程: 导航到拾取点 → 抓取 → 导航到放置点   ║")
    print("╚══════════════════════════════════════════════╝")

    # 启动前置节点（LiDAR / ICP / Nav2 / 串口桥）
    start_prerequisites()

    node = AutoTask()

    print(f"\n[启动] 等待 8s 让节点就绪...")
    time.sleep(8)
    node.publish_initialpose(0, 0)
    time.sleep(1)

    # ── 通知 STM32 自动任务开始 ──
    node.send_auto_cmd(AUTO_CMD_START)
    time.sleep(0.5)

    # ══════════════════════════════════════════════════════
    # 阶段 1: 导航到拾取点 → 检测 → 抓取
    # ══════════════════════════════════════════════════════
    print(f"\n{'═'*50}")
    print(f"  [阶段 1/2] 导航到拾取点 ({PICKUP_X}, {PICKUP_Y})")
    print(f"{'═'*50}")

    node.start_phase("NAV_TO_PICK", PICKUP_X, PICKUP_Y, PICKUP_YAW)

    # 主线程 spin 等待阶段完成（节点在后台接收定位+触发到达+执行抓取）
    print("[等待] 导航 → 到达 → 检测 → 抓取...")
    phase_ok = node.wait_phase(timeout=NAV_TIMEOUT + PICK_TIMEOUT + DETECT_TIMEOUT + 10)
    if phase_ok:
        print("  ✅ 拾取完成")
    else:
        print("  ⚠ 拾取阶段超时或中断，继续下一阶段")
    if not rclpy.ok():
        print("  ⚠ rclpy 已关闭，跳过放置阶段")

    # ══════════════════════════════════════════════════════
    # 阶段 2: 导航到放置点 → 放置
    # ══════════════════════════════════════════════════════
    if rclpy.ok():
        print(f"\n{'═'*50}")
        print(f"  [阶段 2/2] 导航到放置点 ({DROP_X}, {DROP_Y})")
        print(f"{'═'*50}")

        node.start_phase("NAV_TO_DROP", DROP_X, DROP_Y, DROP_YAW)

        print("[等待] 导航 → 到达 → 放置...")
        if node.wait_phase(timeout=NAV_TIMEOUT + PLACE_TIMEOUT + 10):
            print("  ✅ 放置完成")
        else:
            print("  ⚠ 放置阶段超时")

    # ══════════════════════════════════════════════════════
    # 完成
    # ══════════════════════════════════════════════════════
    print(f"\n{'═'*50}")
    print("  自动任务完成 ✅")
    print(f"{'═'*50}")
    if rclpy.ok():
        node.send_auto_cmd(AUTO_CMD_FINISH)
        time.sleep(0.5)

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    cleanup_all()


if __name__ == "__main__":
    main()
