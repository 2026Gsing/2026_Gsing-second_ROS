#!/usr/bin/env python3
"""
box_pick_node.py — 物资箱到达后自动检测 + 重规划 + 抓取

流程：
  1. 在 RViz 中用 2D Nav Goal 导航到物资箱附近 (Terminal 3)
  2. 机器人停稳后自动触发 → 启动 cube_detector.py 检测前方立方体+
  3. 用 catch.py 的坐标变换 + STM32 可达性判断：
     a. 机械臂可达 → 启动 catch.py 抓取
     b. 不可达 → 重新 Nav2 导航靠近，到达后再检测 → 抓取
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
from catch import transform_and_offset, validate_arm_target, stm32_will_accept
import math
import signal
import subprocess
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

        # 等待节点启动 → 自动初始化 ICP 定位（原点, X正向）
        self.get_logger().info("⏳ 等待 8 秒让节点就绪...")
        time.sleep(8)
        self._init_icp_pose()

        self._print_help()

    def _init_icp_pose(self):
        """发布 /initialpose 自动初始化 ICP（原点, X正向）"""
        init_script = str(_HERE / "init_pose.py")
        try:
            subprocess.run(
                ["bash", "-c",
                 f"source /opt/ros/jazzy/setup.bash && "
                 f"RMW_IMPLEMENTATION=rmw_cyclonedds_cpp python3 {init_script}"],
                timeout=10,
            )
            self.get_logger().info("✅ ICP 初始化为 (0,0,0) X正向")
        except Exception as e:
            self.get_logger().warn(f"⚠ ICP 初始化失败: {e}")

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
                    self.get_logger().info(
                        f"[自动到达] 速度归零持续 {ARRIVED_FRAMES} 帧 "
                        f"(v={speed:.3f}m/s x{vx:.3f} y{vy:.3f}) → 启动检测"
                    )
                    self._arrival_triggered = True
                    self.on_arrived()
                elif self._arrival_count % 5 == 0:
                    self.get_logger().info(
                        f"[到达检测] 低速帧 #{self._arrival_count}/{ARRIVED_FRAMES} "
                        f"速度={speed:.4f} m/s"
                    )
            else:
                if self._arrival_count > 0:
                    self.get_logger().info(
                        f"[到达检测] 速度恢复({speed:.3f}m/s)，重置计数 "
                        f"(已积累{self._arrival_count}帧)"
                    )
                self._arrival_count = 0

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

        self.get_logger().info("=" * 60)
        self.get_logger().info(f"[开始] 状态: {self._state} → DETECTING，启动 cube_detector.py")
        self.get_logger().info("=" * 60)

        self._start_cube_detector()
        # 在独立线程中等待检测结果（避免阻塞 spin）
        t = threading.Thread(target=self._detect_loop, daemon=True)
        t.start()

    def _detect_loop(self):
        """等待检测结果，根据距离决定下一步"""
        try:
            # ===== 等待 cube_detector 产出结果 =====
            self.get_logger().info("[检测] 等待 cube_detector 数据（最多 15 秒）...")
            for attempt in range(30):  # 最多等 15 秒
                time.sleep(0.5)
                with self._lock:
                    has_cube = (self._latest_cube is not None
                                and time.monotonic() - self._cube_received_time < 3.0)
                    if has_cube:
                        cube = self._latest_cube
                        self.get_logger().info(f"[检测] 第{attempt+1}/30 帧收到数据")
                        break
                if attempt % 10 == 9:
                    self.get_logger().info(f"[检测] 等待中...({(attempt+1)*0.5:.0f}s)")
            else:
                self.get_logger().warn("[检测] ⏰ 超时: 15 秒未检测到立方体")
                self._cleanup_detection()
                return

            cx, cy, cz = cube.pose.position.x, cube.pose.position.y, cube.pose.position.z
            self.get_logger().info(
                f"[原始] 雷达系坐标: x={cx:.3f}  y={cy:.3f}  z={cz:.3f}  "
                f"xy距离={math.hypot(cx, cy):.3f}m"
            )

            arm_x, arm_y, arm_z, _ = transform_and_offset(cx, cy, cz)
            self.get_logger().info(
                f"[变换] 机械臂系(未补偿): x={arm_x:.3f}  y={arm_y:.3f}  z={arm_z:.3f}  "
                f"距离={math.hypot(arm_x, arm_y, arm_z):.3f}m"
            )

            arm_x += 0.125  # HALF_BOX_HEIGHT 中心→顶面
            self.get_logger().info(
                f"[补偿] 加半高后: x={arm_x:.3f}  y={arm_y:.3f}  z={arm_z:.3f}"
            )

            reachable, reason = validate_arm_target(arm_x, arm_y, arm_z)
            self.get_logger().info(
                f"[ROS侧] 轴边界+IK检查: {'通过' if reachable else f'拒绝({reason})'}"
            )

            if reachable:
                stm32_ok, stm32_reason = stm32_will_accept(arm_x, arm_y, arm_z)
                self.get_logger().info(
                    f"[STM32] 补偿后检查: {'通过' if stm32_ok else f'拒绝({stm32_reason})'}"
                )
                if not stm32_ok:
                    reachable = False
                    reason = f"STM32拒绝({stm32_reason})"

            if reachable:
                self.get_logger().info("→ 判定: 可达，直接抓取")
                self._do_pick()
            else:
                self.get_logger().info(f"→ 判定: 不可达({reason})，靠近重试")
                self._do_renav(cx, cy)

        except Exception as e:
            self.get_logger().error(f"[检测异常] {e}")
            self._cleanup_detection()

    def _do_pick(self):
        """机械臂可达 → 启动 catch.py 抓取，完成后自动清理"""
        with self._lock:
            self._state = "PICKING"
        self.get_logger().info("[抓取] ✓ 可达，启动 catch.py")
        self.get_logger().info("[抓取] catch.py 流程: 稳定检测(3-5s) → AUTO_CMD推进 → 发坐标 → 自动退出")
        self._start_catch()
        # 独立线程等待 catch.py 退出，然后自动清理
        t = threading.Thread(target=self._monitor_catch, daemon=True)
        t.start()

    def _monitor_catch(self):
        """等待 catch.py 退出，自动回到 IDLE"""
        proc = self._catch_proc
        if proc is None:
            return
        self.get_logger().info(f"[抓取] 等待 catch.py (PID={proc.pid}) 完成...")
        try:
            proc.wait(timeout=30)
            self.get_logger().info("[抓取] catch.py 已正常退出")
        except Exception as e:
            self.get_logger().warn(f"[抓取] catch.py 等待异常: {e}")
            try:
                proc.kill()
                proc.wait(timeout=3)
            except Exception:
                pass
        time.sleep(0.5)  # 等机械臂动作收尾
        self.get_logger().info("[抓取] ✅ 抓取流程完成，回到 IDLE")
        self._cleanup_detection()

    def _do_renav(self, cx, cy):
        """机械臂不可达 → 靠近后再检测"""
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

        # 导航到立方体前方 0.3m（沿机器人→立方体方向偏移）
        # 这样不论机器人从哪个方向接近，停稳后立方体都在正前方
        APPROACH_OFFSET = 0.3
        dx = cube_mx - rx
        dy = cube_my - ry
        approach_len = math.hypot(dx, dy)
        if approach_len > 0.01:
            approach_mx = cube_mx - APPROACH_OFFSET * dx / approach_len
            approach_my = cube_my - APPROACH_OFFSET * dy / approach_len
        else:
            approach_mx, approach_my = cube_mx, cube_my

        # 目标朝向：面对立方体
        approach_yaw = math.atan2(cube_my - approach_my, cube_mx - approach_mx)

        dist = math.hypot(cx, cy)
        self.get_logger().info(
            f"[重规划] 立方体 lidar_xy={dist:.3f}m "
            f"→ 导航到前方 {APPROACH_OFFSET:.1f}m 处 "
            f"({approach_mx:.3f}, {approach_my:.3f}) "
            f"朝向={approach_yaw:.2f}rad"
        )
        self._send_nav_goal(approach_mx, approach_my, approach_yaw)

        # ===== 等待导航到达 =====
        self.get_logger().info("[重规划] 等待导航到达（最多 60 秒）...")
        for i in range(120):  # 最多等 60 秒
            time.sleep(0.5)
            with self._lock:
                arrived = self._nav_succeeded
            if arrived:
                self.get_logger().info(f"[重规划] 导航到达(耗时约{i * 0.5:.0f}s)")
                break
            if i % 20 == 19:  # 每 10 秒报一次进度
                self.get_logger().info(f"[重规划] 导航中...({i * 0.5:.0f}s)")
        else:
            self.get_logger().warn("[重规划] 导航超时(60s)")
            self._cleanup_detection()
            return

        self.get_logger().info("[重规划] 已到达，重新检测立方体")
        with self._lock:
            self._nav_succeeded = False

        # ===== 再次检测（用同样的可达性判断）=====
        self.get_logger().info("[二次检测] 等待 cube_detector 数据（最多 5 秒）...")
        for attempt in range(10):  # 等 5 秒
            time.sleep(0.5)
            with self._lock:
                has_cube = (self._latest_cube is not None
                            and time.monotonic() - self._cube_received_time < 3.0)
                if has_cube:
                    cube2 = self._latest_cube
                    self.get_logger().info(f"[二次检测] 第{attempt+1}/10 帧收到数据")
                    break
        if has_cube:
            cx2, cy2, cz2 = cube2.pose.position.x, cube2.pose.position.y, cube2.pose.position.z
            self.get_logger().info(
                f"[二次检测] 原始雷达坐标: ({cx2:.3f}, {cy2:.3f}, {cz2:.3f})"
            )
            ax2, ay2, az2, _ = transform_and_offset(cx2, cy2, cz2)
            self.get_logger().info(
                f"[二次检测] 机械臂系(未补偿): ({ax2:.3f}, {ay2:.3f}, {az2:.3f})"
            )
            ax2 += 0.125
            reachable2, reason2 = validate_arm_target(ax2, ay2, az2)
            self.get_logger().info(
                f"[二次检测] ROS侧检查: {'通过' if reachable2 else f'拒绝({reason2})'}"
            )
            if reachable2:
                stm32_ok2, stm32_reason2 = stm32_will_accept(ax2, ay2, az2)
                self.get_logger().info(
                    f"[二次检测] STM32侧: {'通过' if stm32_ok2 else f'拒绝({stm32_reason2})'}"
                )
                if not stm32_ok2:
                    reachable2 = False
                    reason2 = stm32_reason2
            if reachable2:
                self.get_logger().info("[二次检测] ✓ 可达，启动抓取")
                self._do_pick()
                return
            else:
                self.get_logger().warn(f"[二次检测] ✗ 仍不可达({reason2})，请手动调整")

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
def _sigint_handler(sig, frame):
    """Ctrl+C → 关闭 rclpy，让 spin 返回"""
    rclpy.shutdown()


def main():
    signal.signal(signal.SIGINT, _sigint_handler)
    rclpy.init()
    node = BoxPickNode()

    print("\n" + "=" * 50)
    print("  BoxPickNode — 物资箱到达检测 + 抓取")
    print("=" * 50)
    print("  1. 在 RViz 中用 2D Nav Goal 导航到物资箱")
    print("  2. 机器人停稳后 自动 启动 cube_detector 检测")
    print("     → 机械臂可达则抓取 / 不可达则靠近再检测")
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
