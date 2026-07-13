#!/usr/bin/env python3
"""
box_pick_node.py — 物资箱到达后自动检测 + 重规划 + 抓取

流程：
  1. 在 RViz 中用 2D Nav Goal 导航到物资箱附近 (Terminal 3)
  2. 机器人停稳后自动触发 → 启动 cube_detector.py 检测前方立方体+
  3. 检查 /detected_cube 的 xy 距离：
     a. ≤ 30cm → 启动 catch.py 抓取
     b. > 30cm → 重新 Nav2 导航到立方体位置，到达后再检测 → 抓取
  4. 完成后回到 IDLE，可开始下一箱
  5. 也可手动输入 arrived（自动检测未生效时备用）

启动（自动拉起 LiDAR、ICP、Nav2、串口桥等前置节点）：
  python3 py/control/box_pick_node.py
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from arrival_detector import quaternion_to_yaw
from launch_utils import start_prerequisites, cleanup_all
import math
import signal
import sys
import time
import threading
from pathlib import Path

_HERE = Path(__file__).resolve().parent          # py/control/

# ============ 自动到达检测参数 ============
VEL_STOP_THRESHOLD = 0.03     # 低于此速度视为停止 (m/s)
ARRIVED_FRAMES = 20           # 连续多少帧速度低于阈值算到达 (~2s @10Hz)


class BoxPickNode(Node):
    def __init__(self):
        super().__init__('box_pick_node')

        # ============ 线程锁（保护多线程竞争） ============
        self._lock = threading.Lock()

        # ============ 子进程 ============
        self._cube_proc = None
        self._catch_proc = None

        # ============ 数据 ============
        self._latest_localization = None   # Odometry msg (map 坐标系)
        self._latest_cube = None           # Marker msg (unilidar_lidar 坐标)
        self._cube_received_time = 0.0

        # ============ 自动到达检测 ============
        self._arrival_count = 0            # 连续低速帧计数
        self._arrival_triggered = False    # 是否已触发到达流程（防重复）

        # ============ 状态 ============
        self._busy = False
        self._state = "IDLE"               # IDLE / DETECTING / RENAV / PICKING

        # ============ 订阅 ============
        self.create_subscription(Marker, '/detected_cube', self._cube_cb, 10)
        self.create_subscription(Odometry, '/localization', self._localization_cb, 10)

        # ============ Nav2 Action Client ============
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._nav_goal_handle = None
        self._nav_succeeded = False

        # 自动启动前置 ROS 节点
        start_prerequisites()
        self.get_logger().info("BoxPickNode 已启动")
        self._print_help()

    # ================================================================
    # CLI 帮助
    # ================================================================
    def _print_help(self):
        print("\n── 命令 ──────────────────────")
        print("  arrived   → 手动确认到达（自动检测失效时备用）")
        print("  status    → 显示当前状态")
        print("  stop      → 停止所有子进程")
        print("  quit      → 退出")
        print("──────────────────────────────\n")

    # ================================================================
    # 回调
    # ================================================================
    def _cube_cb(self, msg):
        with self._lock:
            self._latest_cube = msg
            self._cube_received_time = time.monotonic()

    def _localization_cb(self, msg):
        with self._lock:
            self._latest_localization = msg

            # ============ 自动到达检测 ============
            if self._arrival_triggered or self._busy:
                return  # 已触发或正在处理，跳过

            vx = msg.twist.twist.linear.x
            vy = msg.twist.twist.linear.y
            speed = math.hypot(vx, vy)

            if speed < VEL_STOP_THRESHOLD:
                self._arrival_count += 1
                if self._arrival_count >= ARRIVED_FRAMES:
                    self.get_logger().info(f"[自动到达] 速度归零持续 {ARRIVED_FRAMES} 帧，启动检测")
                    self._arrival_triggered = True
                    self.on_arrived()
            else:
                self._arrival_count = 0  # 有速度了，重置计数

    # ================================================================
    # Nav2 导航回调
    # ================================================================
    def _nav_goal_response_cb(self, future):
        goal_handle = future.result()
        with self._lock:
            if not goal_handle.accepted:
                self.get_logger().warn("[NAV] 目标被拒绝")
                self._nav_goal_handle = None
                return
            self._nav_goal_handle = goal_handle
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self._nav_result_cb)

    def _nav_result_cb(self, future):
        result = future.result()
        with self._lock:
            if result and result.status == 4:  # SUCCEEDED
                self._nav_succeeded = True
                self.get_logger().info("[NAV] 导航成功！")
            self._nav_goal_handle = None

    def _send_nav_goal(self, x, y, yaw):
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

        if not self._nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("[NAV] Nav2 action server 不可用")
            return False

        with self._lock:
            self._nav_succeeded = False
        send_future = self._nav_client.send_goal_async(goal)
        send_future.add_done_callback(self._nav_goal_response_cb)
        self.get_logger().info(f"[NAV] 发送目标: ({x:.2f}, {y:.2f}, {yaw:.2f})")
        return True

    # ================================================================
    # 主流程：到达 → 检测 → 抓取 / 重规划
    # ================================================================
    def on_arrived(self):
        """用户确认已到达物资箱附近，开始检测流程"""
        with self._lock:
            if self._busy:
                self.get_logger().warn("[!] 正在处理中，请等待或输入 stop")
                return
            self._busy = True
            self._state = "DETECTING"
            self._nav_succeeded = False

        self.get_logger().info("=" * 50)
        self.get_logger().info("[开始] 启动 cube_detector.py ...")
        self.get_logger().info("=" * 50)

        self._start_cube_detector()
        # 在独立线程中等待检测结果（避免阻塞 spin）
        t = threading.Thread(target=self._detect_loop, daemon=True)
        t.start()

    def _detect_loop(self):
        """等待检测结果，根据距离决定下一步"""
        try:
            # ===== 等待 cube_detector 产出结果 =====
            for attempt in range(30):  # 最多等 15 秒
                time.sleep(0.5)
                with self._lock:
                    has_cube = (self._latest_cube is not None
                                and time.monotonic() - self._cube_received_time < 3.0)
                    if has_cube:
                        cube = self._latest_cube
                        break
            else:
                self.get_logger().warn("[超时] 15 秒未检测到立方体")
                self._cleanup_detection()
                return

            cx = cube.pose.position.x
            cy = cube.pose.position.y
            xy_dist = math.hypot(cx, cy)
            self.get_logger().info(f"[检测] 立方体: x={cx:.3f}  y={cy:.3f}  xy距离={xy_dist:.3f}m")

            if xy_dist <= DIST_XY_NEAR:
                self._do_pick()
            else:
                self._do_renav(cx, cy)

        except Exception as e:
            self.get_logger().error(f"[检测异常] {e}")
            self._cleanup_detection()

    def _do_pick(self):
        """xy ≤ 30cm：直接启动 catch.py 抓取"""
        with self._lock:
            self._state = "PICKING"
        self.get_logger().info(f"[抓取] 立方体在 {DIST_XY_NEAR}m 内，启动 catch.py")
        self._start_catch()
        # catch.py 会持续运行（稳定检测 + 串口发送），
        # 用户可在抓取完成后输入 stop 停止
        with self._lock:
            self._busy = False

    def _do_renav(self, cx, cy):
        """xy > 30cm：重新导航到立方体位置"""
        with self._lock:
            self._state = "RENAV"
            loc = self._latest_localization

        if loc is None:
            self.get_logger().error("[重规划] 无定位数据，无法计算目标")
            self._cleanup_detection()
            return

        # 立方体坐标从 lidar 系 → map 系
        robot = loc.pose.pose
        yaw = quaternion_to_yaw(robot.orientation)
        rx = robot.position.x
        ry = robot.position.y

        cube_mx = rx + cx * math.cos(yaw) - cy * math.sin(yaw)
        cube_my = ry + cx * math.sin(yaw) + cy * math.cos(yaw)

        dist = math.hypot(cx, cy)
        self.get_logger().info(f"[重规划] 立方体在 {dist:.3f}m 外，导航到 ({cube_mx:.3f}, {cube_my:.3f})")
        self._send_nav_goal(cube_mx, cube_my, yaw)

        # ===== 等待导航到达 =====
        for _ in range(120):  # 最多等 60 秒
            time.sleep(0.5)
            with self._lock:
                arrived = self._nav_succeeded
            if arrived:
                break
        else:
            self.get_logger().warn("[重规划] 导航超时")
            self._cleanup_detection()
            return

        self.get_logger().info("[重规划] 已到达，再次检测立方体")
        with self._lock:
            self._nav_succeeded = False

        # ===== 再次检测 =====
        for attempt in range(10):  # 等 5 秒
            time.sleep(0.5)
            with self._lock:
                has_cube = (self._latest_cube is not None
                            and time.monotonic() - self._cube_received_time < 3.0)
                if has_cube:
                    cube2 = self._latest_cube
                    break
            if has_cube:
                cx2 = cube2.pose.position.x
                cy2 = cube2.pose.position.y
                xy_dist2 = math.hypot(cx2, cy2)
                self.get_logger().info(f"[二次检测] 立方体: x={cx2:.3f}  y={cy2:.3f}  xy距离={xy_dist2:.3f}m")

                if xy_dist2 <= DIST_XY_NEAR:
                    self._do_pick()
                    return
                else:
                    self.get_logger().warn(f"[二次检测] 仍在 {DIST_XY_NEAR}m 外，请手动调整")
                    break

        self.get_logger().warn("[重规划] 二次检测未抓到，回到 IDLE")
        self._cleanup_detection()

    # ================================================================
    # 子进程管理
    # ================================================================
    def _start_cube_detector(self):
        if self._cube_proc is not None:
            return
        script = str(_HERE / "cube_detector.py")
        try:
            self._cube_proc = subprocess.Popen(
                [sys.executable, script],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            self.get_logger().info(f"[CUBE] cube_detector PID={self._cube_proc.pid}")
        except Exception as e:
            self.get_logger().error(f"[CUBE] 启动失败: {e}")

    def _stop_cube_detector(self):
        if self._cube_proc is None:
            return
        try:
            self._cube_proc.send_signal(signal.SIGINT)
            self._cube_proc.wait(timeout=3.0)
        except Exception:
            try:
                self._cube_proc.kill()
                self._cube_proc.wait()
            except Exception:
                pass
        self._cube_proc = None
        self.get_logger().info("[CUBE] 已停止")

    def _start_catch(self):
        if self._catch_proc is not None:
            return
        script = str(_HERE / "catch.py")
        try:
            self._catch_proc = subprocess.Popen(
                [sys.executable, script],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            self.get_logger().info(f"[CATCH] catch.py PID={self._catch_proc.pid}")
        except Exception as e:
            self.get_logger().error(f"[CATCH] 启动失败: {e}")

    def _stop_catch(self):
        if self._catch_proc is None:
            return
        try:
            self._catch_proc.send_signal(signal.SIGINT)
            self._catch_proc.wait(timeout=3.0)
        except Exception:
            try:
                self._catch_proc.kill()
                self._catch_proc.wait()
            except Exception:
                pass
        self._catch_proc = None
        self.get_logger().info("[CATCH] 已停止")

    def _cleanup_detection(self):
        """停止所有子进程，重置状态"""
        self._stop_cube_detector()
        self._stop_catch()
        with self._lock:
            self._state = "IDLE"
            self._busy = False
            self._nav_succeeded = False
            self._latest_cube = None
            self._arrival_triggered = False  # 允许下次自动到达检测
            self._arrival_count = 0
        self.get_logger().info("[结束] 已回到 IDLE，可开始下一箱")

    # ================================================================
    # CLI 命令
    # ================================================================
    def cmd_status(self):
        with self._lock:
            s = f"状态: {self._state}  busy={'是' if self._busy else '否'}"
            if self._latest_cube:
                c = self._latest_cube.pose.position
                d = math.hypot(c.x, c.y)
                s += f"  最近立方体: ({c.x:.3f},{c.y:.3f}) xy距离={d:.3f}m"
            else:
                s += "  无立方体数据"
        self.get_logger().info(s)

    def cmd_stop(self):
        self._cleanup_detection()
        self.get_logger().info("[命令] 已停止所有")


# ================================================================
# 入口
# ================================================================
def main():
    rclpy.init()
    node = BoxPickNode()

    print("\n" + "=" * 50)
    print("  BoxPickNode — 物资箱到达检测 + 抓取")
    print("=" * 50)
    print("  1. 在 RViz 中用 2D Nav Goal 导航到物资箱")
    print("  2. 机器人停稳后 自动 启动 cube_detector 检测")
    print("     → 30cm内抓取 / 远处重规划导航")
    print("  3. 输入 arrived 可手动触发（自动检测未生效时备用）")
    print("=" * 50 + "\n")

    # CLI 输入线程
    def cli_thread():
        while rclpy.ok():
            try:
                cmd = input().strip().lower()
                if cmd == "arrived":
                    node.on_arrived()
                elif cmd == "status":
                    node.cmd_status()
                elif cmd == "stop":
                    node.cmd_stop()
                elif cmd == "quit":
                    rclpy.shutdown()
                    break
            except (EOFError, KeyboardInterrupt):
                break

    t = threading.Thread(target=cli_thread, daemon=True)
    t.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._cleanup_detection()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
