#!/usr/bin/env python3
"""
open_loop_auto.py — 开环自动控制脚本（无 Nav2 定位）

流程:
  1. 开门禁 (AUTO_CMD_START)
  2. 前进 3s → 接近物资箱
  3. 启动 cube_detector.py + catch.py 检测并抓取
  4. 前进 6s → 携带物资箱到放置区
  5. 放下物资箱（按抓取位置）
  6. 完成

与 auto_task.py 的区别：
  - 不使用 Nav2 导航，全靠开环速度控制
  - 不需要地图，不需要 ICP 定位
  - 只需要 LiDAR + 串口桥

使用方式:
./ros-run.sh py/control/open_loop_auto.py
"""

import argparse
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
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from visualization_msgs.msg import Marker

_HERE = Path(__file__).resolve().parent
_PROJECT = _HERE.parent.parent

sys.path.insert(0, str(_HERE))
from launch_utils import launch, cleanup_all, _kill_existing, _clean_cyclone_shm
import catch as _catch  # 坐标变换 + 工作空间验证（force-grab 用）

# ============================================================
# 参数（可被 --vx / --approach / --carry 覆盖）
# ============================================================
FORWARD_SPEED = 0.2        # 前进速度 (m/s)
APPROACH_TIME = 10        # 第一次前进持续时间 (s)
CARRY_TIME = 20.0           # 第二次前进持续时间 (s)
GRAB_TIMEOUT = 100.0        # 抓取超时 (s)
PLACE_TIMEOUT = 60.0       # 放置超时 (s)

# ============================================================
# 距离阈值 + 机械臂最远可达坐标（雷达系）
# ============================================================
DISTANCE_THRESHOLD = 0.35  # 若 cube x > 此值，认为太远需夹爪坐标 (m)
ARM_MAX_REACH_X = 0.35     # 雷达系 x 最远可达（前向，m）
ARM_MAX_REACH_Y = 0.15     # 雷达系 |y| 最远可达（侧向，m）

# ============================================================
# 串口协议常量（与 STM32 / catch.py / auto_task.py 一致）
# ============================================================
AUTO_CMD_START = 1
AUTO_CMD_ARRIVED_BOX = 2
AUTO_CMD_ARRIVED_ZONE = 4
AUTO_CMD_FINISH = 7

ARM_MISSION_BACK_TO_PLACE = 2
ARM_MISSION_HAS_PLACE = 0x04

# 放置坐标（机械臂系）
# arm_x(高度), arm_y(侧向), arm_z(前向)
PLACE_X = -0.25   # 向下放
PLACE_Y = 0.0     # 正中
PLACE_Z = -0.35   # 臂向前伸


class OpenLoopAutoNode(Node):
    """开环自动控制节点"""

    def __init__(self):
        super().__init__("open_loop_auto")

        # ==================== 发布器 ====================
        self.vel_pub = self.create_publisher(Twist, "/vision_cmd_vel", 10)
        self.auto_pub = self.create_publisher(String, "/vision/auto_cmd", 10)
        self.arm_pub = self.create_publisher(String, "/vision/arm_mission", 10)

        # ==================== 订阅 cube 检测（仅日志） ====================
        self._detected_cube = None
        self.create_subscription(Marker, "/detected_cube", self._cube_cb, 10)

        # ==================== 50Hz 速度重发机制 ====================
        # 防止 STM32 串口桥的 500ms 视觉超时自动停车
        self._vx = 0.0
        self._wz = 0.0
        self.create_timer(0.02, self._republish_cb)

        # ==================== 子进程 ====================
        self._cube_proc = None
        self._catch_proc = None
        self._subprocess_lock = threading.Lock()

        # ==================== 后台 spin 线程 ====================
        threading.Thread(target=self._bg_spin, daemon=True).start()

        # ==================== 启动前置节点 ====================
        # 不再在 __init__ 中启动，由 main() 调用 standalone startup()

        self.get_logger().info("=" * 60)
        self.get_logger().info("  开环自动控制脚本")
        self.get_logger().info(f"  流程: 开门 → 前进 {APPROACH_TIME}s → 抓取 "
                               f"→ 前进 {CARRY_TIME}s → 放置")
        self.get_logger().info("=" * 60)

    # ──────────────────────────────────────────────────────────────
    # 回调
    # ──────────────────────────────────────────────────────────────

    def _cube_cb(self, msg):
        """记录检测到的立方体（首次检测或位置变化时输出日志）"""
        was_none = self._detected_cube is None
        self._detected_cube = msg
        if was_none:
            # 首次检测到立方体
            p = msg.pose.position
            s = msg.scale
            self.get_logger().info(
                f"[检测] 🎯 首次检测到物资箱! "
                f"位置: ({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) "
                f"尺寸: ({s.x:.3f}, {s.y:.3f}, {s.z:.3f})"
            )
        elif msg.header.stamp.sec % 2 == 0:
            # 每 ~2s 输出一次当前检测状态（避免刷屏）
            p = msg.pose.position
            self.get_logger().info(
                f"[检测] 📦 物资箱: x={p.x:.3f} y={p.y:.3f} z={p.z:.3f}"
            )

    def _bg_spin(self):
        """后台 spin 线程"""
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

    def _republish_cb(self):
        """50Hz 持续重发当前目标速度，防止 500ms 视觉超时停车"""
        msg = Twist()
        msg.linear.x = self._vx
        msg.angular.z = self._wz
        self.vel_pub.publish(msg)

    # ──────────────────────────────────────────────────────────────
    # 速度控制
    # ──────────────────────────────────────────────────────────────

    def set_vel(self, vx, wz=0.0):
        """设置目标速度（定时器以 50Hz 持续重发）"""
        self._vx = float(vx)
        self._wz = float(wz)
        # 立即发一次，避免等待定时器下次触发
        msg = Twist()
        msg.linear.x = self._vx
        msg.angular.z = self._wz
        self.vel_pub.publish(msg)
        self.get_logger().info(f"[VEL] vx={vx:+.2f}  wz={wz:+.2f}")

    def stop(self):
        """停止运动"""
        self.set_vel(0.0, 0.0)
        self.get_logger().info("[VEL] 停止")

    def move_open_loop(self, duration, speed=None):
        """
        开环前进指定时长。

        在移动过程中持续 spin 以保持回调处理。
        """
        if speed is None:
            speed = FORWARD_SPEED
        self.get_logger().info(f"[MOVE] 前进 {duration:.1f}s @ {speed:.2f}m/s")
        self.set_vel(speed, 0.0)
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline and rclpy.ok():
            time.sleep(0.05)  # 由 _bg_spin 线程处理 spin
        self.stop()

    def _force_grab_prepare(self):
        """
        停止 cube_detector 并发布夹爪坐标。

        用 catch.py 的坐标变换（雷达系→机械臂系），若超出工作空间则
        按比例缩小 arm_z 和 arm_y（保持 arm_x 高度不变），使方向不变
        但距离缩至机械臂可达范围。
        最后逆变换回雷达系发布，catch.py 能直接接受。
        """
        p = self._detected_cube.pose.position
        rx, ry, rz = p.x, p.y, p.z

        # 1) 雷达系 → 机械臂系
        arm_x, arm_y, arm_z, _ = _catch.transform_and_offset(rx, ry, rz)
        arm_x += _catch.HALF_BOX_HEIGHT  # 中心 → 顶面

        orig_arm = (arm_x, arm_y, arm_z)

        # 2) 若 dist 超限，按比例缩小 arm_z 和 arm_y（保持 arm_x 高度）
        dist = math.sqrt(arm_x * arm_x + arm_y * arm_y + arm_z * arm_z)
        if dist > _catch.ARM_DIST_MAX:
            # 高度不变，缩小水平分量
            h2 = arm_y * arm_y + arm_z * arm_z           # 水平分量平方
            max_h2 = _catch.ARM_DIST_MAX * _catch.ARM_DIST_MAX - arm_x * arm_x
            if max_h2 > 0 and h2 > 0:
                scale = math.sqrt(max_h2 / h2)
                arm_y *= scale
                arm_z *= scale

        # 3) 各轴限幅（保险）
        arm_x = max(_catch.ARM_TARGET_X_MIN, min(arm_x, _catch.ARM_TARGET_X_MAX))
        arm_y = max(_catch.ARM_TARGET_Y_MIN, min(arm_y, _catch.ARM_TARGET_Y_MAX))
        arm_z = max(_catch.ARM_TARGET_Z_MIN, min(arm_z, _catch.ARM_TARGET_Z_MAX))

        # 4) 最终验证
        valid, reason = _catch.validate_arm_target(arm_x, arm_y, arm_z)
        stm32_ok, stm32_reason = _catch.stm32_will_accept(arm_x, arm_y, arm_z)
        if not (valid and stm32_ok):
            self.get_logger().error(
                f"[强制抓取] ❌ 修正后仍无法通过验证: {reason} / {stm32_reason}"
            )
            return  # 放弃强制抓取，后续 catch.py 会收到原始坐标

        # 5) 逆变换回雷达系（catch.py 读了会用同一套变换算出上面的 arm 坐标）
        clamped_rx = -arm_z - _catch.OFFSET_Z
        clamped_ry = arm_y
        clamped_rz = -arm_x - _catch.OFFSET_X + _catch.HALF_BOX_HEIGHT

        self.get_logger().info(
            f"[强制抓取] 🔧 雷达 ({rx:.3f},{ry:.3f},{rz:.3f}) "
            f"→ arm ({orig_arm[0]:.3f},{orig_arm[1]:.3f},{orig_arm[2]:.3f}) "
            f"→ 修正 ({arm_x:.3f},{arm_y:.3f},{arm_z:.3f}) "
            f"→ 雷达 ({clamped_rx:.3f},{clamped_ry:.3f},{clamped_rz:.3f})"
        )

        # 停止 cube_detector（避免原始坐标干扰 catch.py 的稳定检测）
        with self._subprocess_lock:
            if self._cube_proc is not None:
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

        # 修改检测坐标为逆变换后的最远可达值
        self._detected_cube.pose.position.x = clamped_rx
        self._detected_cube.pose.position.y = clamped_ry
        self._detected_cube.pose.position.z = clamped_rz

        # 持续发布修正坐标（catch.py 的 cube_callback 需要连续稳定帧）
        self._override_pub = self.create_publisher(Marker, "/detected_cube", 10)
        self._override_timer = self.create_timer(0.1, self._publish_override_cube)

    def _publish_override_cube(self):
        """定时器回调：持续发布修正后的 marker（直至 catch.py 完成抓取）"""
        self._override_pub.publish(self._detected_cube)

    # ──────────────────────────────────────────────────────────────
    # 串口指令
    # ──────────────────────────────────────────────────────────────

    def send_auto_cmd(self, cmd, target=0, zone=0):
        """发送 0x15 自动任务命令（通过串口桥）"""
        payload = json.dumps({"cmd": cmd, "target": target, "zone": zone})
        self.auto_pub.publish(String(data=payload))
        cmd_names = {1: "START", 2: "ARRIVED_BOX", 4: "ARRIVED_ZONE", 7: "FINISH"}
        name = cmd_names.get(cmd, f"0x{cmd:02x}")
        self.get_logger().info(f"[AUTO] {name} cmd={cmd} target={target} zone={zone}")

    def send_arm_mission(self, mode, flags, sequences):
        """
        发送 0x14 机械臂任务（通过串口桥）。

        Args:
            mode: 任务模式 (1=PICK_TO_BACK, 2=BACK_TO_PLACE)
            flags: 位标志 (0x01=HAS_PICK, 0x02=HAS_BACK, 0x04=HAS_PLACE)
            sequences: dict 含 pick/back/place 坐标
        """
        payload = {"mode": mode, "flags": flags}
        for key in ("pick", "back", "place"):
            if key in sequences:
                payload[key] = sequences[key]
        self.arm_pub.publish(String(data=json.dumps(payload)))
        mode_names = {1: "PICK_TO_BACK", 2: "BACK_TO_PLACE"}
        name = mode_names.get(mode, f"mode={mode}")
        self.get_logger().info(f"[ARM] {name} flags={flags}")

    # ──────────────────────────────────────────────────────────────
    # 子进程管理（带日志实时转发）
    # ──────────────────────────────────────────────────────────────

    def _start_subprocess_with_relay(self, script_relpath, log_dir, file_prefix, tag):
        """
        启动子进程，将 stdout 同时写入日志文件和转发到 ROS logger。

        Args:
            script_relpath: 脚本路径（相对于 _HERE）
            log_dir: 日志目录（Path 对象）
            file_prefix: 日志文件名前缀
            tag: ROS logger 标签（如 "CUBE", "CATCH"）

        Returns:
            subprocess.Popen 对象，或 None（启动失败时）
        """
        log_dir = Path(log_dir)
        log_dir.mkdir(parents=True, exist_ok=True)
        logfile = str(log_dir / f"{time.strftime('%Y-%m-%d_%H%M%S')}_{file_prefix}.log")

        try:
            proc = subprocess.Popen(
                [sys.executable, str(_HERE / script_relpath)],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                bufsize=1,        # 行缓冲
                text=True,        # 文本模式 → line 是 str
            )
        except Exception as e:
            self.get_logger().error(f"[{tag}] 启动失败: {e}")
            return None

        def _relay():
            """后台线程：逐行读取子进程 stdout，写文件 + 转发 ROS logger"""
            try:
                with open(logfile, "w") as f:
                    for line in iter(proc.stdout.readline, ""):
                        f.write(line)
                        f.flush()
                        line = line.rstrip("\n\r")
                        if line:
                            self.get_logger().info(f"[{tag}] {line}")
            except Exception:
                pass

        thread = threading.Thread(target=_relay, daemon=True)
        thread.start()

        self.get_logger().info(f"[{tag}] PID={proc.pid} → {logfile}")
        return proc

    def _start_cube_detector(self):
        """启动 cube_detector.py 子进程（输出实时转发到终端）"""
        with self._subprocess_lock:
            if self._cube_proc is not None:
                return
            self._cube_proc = self._start_subprocess_with_relay(
                "cube_detector.py",
                _PROJECT / "logs" / "vision",
                "cube_detector",
                "CUBE",
            )

    def _start_catch(self):
        """启动 catch.py 子进程（输出实时转发到终端）"""
        with self._subprocess_lock:
            if self._catch_proc is not None:
                return
            self._catch_proc = self._start_subprocess_with_relay(
                "catch.py",
                _PROJECT / "logs" / "arm",
                "catch",
                "CATCH",
            )

    def _cleanup_subprocesses(self):
        """停止 cube_detector 和 catch 子进程"""
        with self._subprocess_lock:
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

    # ──────────────────────────────────────────────────────────────
    # 主流程步骤
    # ──────────────────────────────────────────────────────────────

    def step_open_gate(self):
        """[步骤 1] 开门禁 — 发送 START 允许底盘运动"""
        self.get_logger().info("")
        self.get_logger().info("╔══════════════════════════════════════╗")
        self.get_logger().info("║  步骤 1/5: 开门禁 (AUTO_CMD_START)  ║")
        self.get_logger().info("╚══════════════════════════════════════╝")
        self.send_auto_cmd(AUTO_CMD_START)
        time.sleep(0.3)
        self.get_logger().info("[OK] 门禁已开")

    def step_approach(self):
        """[步骤 2] 前进指定时长接近物资箱"""
        self.get_logger().info("")
        self.get_logger().info("╔══════════════════════════════════════╗")
        self.get_logger().info(f"║  步骤 2/5: 前进 {APPROACH_TIME}s 接近物资箱  ║")
        self.get_logger().info("╚══════════════════════════════════════╝")
        self.move_open_loop(APPROACH_TIME)
        self.get_logger().info("[OK] 到达物资箱区域")

    def step_grab(self):
        """[步骤 3] 检测并抓取物资箱（太远时用最远可达坐标）"""
        self.get_logger().info("")
        self.get_logger().info("╔══════════════════════════════════════╗")
        self.get_logger().info("║  步骤 3/5: 检测并抓取物资箱         ║")
        self.get_logger().info("╚══════════════════════════════════════╝")

        # ── 启动 cube_detector（先启动，让它积累点云帧） ──
        self._start_cube_detector()
        self.get_logger().info("[抓取] 等待 cube_detector 初始化 (1.5s)...")
        time.sleep(1.5)

        # ── 等待检测到物资箱（5s 超时） ──
        self.get_logger().info("[抓取] 等待检测物资箱...")
        _wait_deadline = time.monotonic() + 5.0
        while time.monotonic() < _wait_deadline and rclpy.ok() and self._detected_cube is None:
            time.sleep(0.1)  # 由 _bg_spin 线程处理 spin

        # ── 检查距离，太远则用最远可达坐标（根据 catch.py 的工作空间限制） ──
        # cube_detector 已改为选最近的立方体发布
        if self._detected_cube is not None:
            cube_x = self._detected_cube.pose.position.x
            self.get_logger().info(f"[抓取] 📦 物资箱 x 距离: {cube_x:.3f}m")

            if cube_x > DISTANCE_THRESHOLD:
                self.get_logger().info(
                    f"[抓取] 🔧 太远 (x={cube_x:.3f}m > {DISTANCE_THRESHOLD}m)，"
                    f"使用最远可达坐标 (z不变)"
                )
                self._force_grab_prepare()
        else:
            self.get_logger().warn("[抓取] ⚠️ 未检测到物资箱，直接启动 catch.py")

        # ── 启动 catch.py ──
        # catch.py 内部完整流程：
        #   订阅 /detected_cube → 稳定检测（滑动窗口标准差）
        #   → 推进 STM32 状态机 (START → ARRIVED_BOX → PICK)
        #   → 发 0x14 PICK_TO_BACK 机械臂坐标
        #   → 等待 STM32 回传 ARM_EVENT pick_done 后退出
        self._start_catch()

        # ── 等待抓取完成（catch.py 等到 pick_done 才退出） ──
        if self._catch_proc:
            self.get_logger().info(
                f"[抓取] 等待 STM32 抓取完成 (PID={self._catch_proc.pid}) "
                f"超时 {GRAB_TIMEOUT:.0f}s..."
            )

            # 等待期间周期性输出检测状态
            _wait_start = time.monotonic()
            _last_report = 0.0
            try:
                while True:
                    # 非阻塞检查子进程是否退出
                    try:
                        self._catch_proc.wait(timeout=2.0)
                        elapsed = time.monotonic() - _wait_start
                        self.get_logger().info(
                            f"[抓取] ✅ STM32 抓取完成 (耗时 {elapsed:.1f}s)"
                        )
                        break
                    except subprocess.TimeoutExpired:
                        pass

                    # 每 4s 报告一次当前检测状态
                    elapsed = time.monotonic() - _wait_start
                    if elapsed - _last_report >= 4.0:
                        _last_report = elapsed
                        if self._detected_cube is not None:
                            p = self._detected_cube.pose.position
                            self.get_logger().info(
                                f"[抓取] ⏳ 等待中 {elapsed:.0f}s/{GRAB_TIMEOUT:.0f}s "
                                f"| 物资箱 @ x={p.x:.3f} y={p.y:.3f} z={p.z:.3f}"
                            )
                        else:
                            self.get_logger().info(
                                f"[抓取] ⏳ 等待中 {elapsed:.0f}s/{GRAB_TIMEOUT:.0f}s "
                                f"| 未检测到物资箱..."
                            )

                    # 检查是否超时
                    if elapsed > GRAB_TIMEOUT:
                        self.get_logger().warn(
                            f"[抓取] ⏰ STM32 抓取超时 ({GRAB_TIMEOUT:.0f}s)"
                        )
                        try:
                            self._catch_proc.kill()
                            self._catch_proc.wait(timeout=3.0)
                        except Exception:
                            pass
                        break

            except Exception as e:
                self.get_logger().error(f"[抓取] 等待过程异常: {e}")

        # ── 清理 ──
        if hasattr(self, '_override_timer') and self._override_timer is not None:
            self._override_timer.cancel()
            self._override_timer = None
        self._cleanup_subprocesses()
        self.get_logger().info("[抓取] ✅ 抓取阶段完成，继续前进")

    def step_carry(self):
        """[步骤 4] 前进指定时长携带物资箱到放置区"""
        self.get_logger().info("")
        self.get_logger().info("╔══════════════════════════════════════╗")
        self.get_logger().info(f"║  步骤 4/5: 前进 {CARRY_TIME}s 携带到放置区     ║")
        self.get_logger().info("╚══════════════════════════════════════╝")
        self.move_open_loop(CARRY_TIME)
        self.get_logger().info("[OK] 到达放置区")

    def step_place(self):
        """[步骤 5] 放下物资箱"""
        self.get_logger().info("")
        self.get_logger().info("╔══════════════════════════════════════╗")
        self.get_logger().info("║  步骤 5/5: 放下物资箱               ║")
        self.get_logger().info("╚══════════════════════════════════════╝")

        # 推进 STM32 状态机到 PLACE
        # 根据抓取时的箱子坐标选择归位区（有记录则用，无则用 zone=0）
        zone_id = 0
        if self._detected_cube is not None:
            cx = self._detected_cube.pose.position.x
            # 箱子在雷达系中的 x 坐标 ≈ 离机器人距离
            # 距离远 (x > 0.15) 表示箱子在前方区域，归位区 0
            # 可在此处根据实际情况调整 zone 选择逻辑
            zone_id = 0
            self.get_logger().info(
                f"[放置] 抓取位置: x={cx:.3f} (雷达系) → 归位区 {zone_id}"
            )
        else:
            self.get_logger().info("[放置] 无检测记录，默认归位区 0")

        self.send_auto_cmd(AUTO_CMD_ARRIVED_ZONE, target=1, zone=zone_id)
        time.sleep(0.5)

        # 发送放置坐标（与 auto_task.py 一致）
        self.send_arm_mission(
            mode=ARM_MISSION_BACK_TO_PLACE,
            flags=ARM_MISSION_HAS_PLACE,
            sequences={"place": {"x": PLACE_X, "y": PLACE_Y, "z": PLACE_Z}},
        )

        # 等待 STM32 执行放置
        self.get_logger().info(f"[放置] 等待机械臂执行放置（超时 {PLACE_TIMEOUT:.0f}s）...")
        time.sleep(PLACE_TIMEOUT)
        self.get_logger().info("[放置] ✅ 放置完成")

    def finish(self):
        """完成 — 发送 FINISH + 停止 + 清理"""
        self.get_logger().info("")
        self.get_logger().info("╔══════════════════════════════════════╗")
        self.get_logger().info("║  开环自动控制完成 ✅                ║")
        self.get_logger().info("╚══════════════════════════════════════╝")
        self.send_auto_cmd(AUTO_CMD_FINISH)
        self.stop()
        self._cleanup_subprocesses()


# ============================================================
# 启动前置节点（参照 start_prerequisites 的写法）
# ============================================================

def _startup():
    """启动 LiDAR 驱动 + 串口桥（开环控制不需要 ICP/Nav2）"""
    # ── 启动日志文件 ──
    _startup_logdir = _PROJECT / "logs" / "startup"
    _startup_logdir.mkdir(parents=True, exist_ok=True)
    _startup_log = _startup_logdir / f"{time.strftime('%Y-%m-%d_%H%M%S')}_open_loop_startup.log"

    def _log(msg):
        print(msg)
        with open(_startup_log, "a") as f:
            f.write(msg + "\n")

    _log("[启动] 启动前置节点 (LiDAR + 串口桥)...")
    _log(f"[启动] 日志 → {_startup_log}")

    _kill_existing()
    _clean_cyclone_shm()

    # 硬件自动检测
    _hw = {}
    try:
        sys.path.insert(0, str(_PROJECT / "py"))
        from tools.detect_hardware import detect_all
        _hw = detect_all(verbose=False)
        sys.path.pop(0)
        _log(f"[硬件] 串口={_hw.get('serial_port','?')}  LiDAR IP={_hw.get('lidar_ip','?')}")
    except Exception as e:
        _log(f"[硬件] 自动检测失败: {e}")

    _lidar_ip = _hw.get("lidar_ip", "192.168.1.1")
    _local_ip = _hw.get("local_ip", "192.168.1.2")
    _serial_port = _hw.get("serial_port", "/dev/ttyACM0")

    # 检测 ROS2 发行版
    _ros_setup = None
    for _d in ("jazzy", "humble"):
        _p = f"/opt/ros/{_d}/setup.bash"
        if os.path.isfile(_p):
            _ros_setup = f"source {_p}"
            _log(f"[启动] ROS2 发行版: {_d}")
            break
    if _ros_setup is None:
        _ros_setup = "source /opt/ros/humble/setup.bash"
        _log("[启动] ⚠️ 未检测到 ROS2，默认 humble")

    _fastlio_dir = str(_PROJECT / "fastlio2_v2")
    _nav2_dir = str(_PROJECT / "nav2_ws1")

    # ── LiDAR 驱动 ──
    _log("[启动] 启动 LiDAR 驱动...")
    launch(
        f"cd {_fastlio_dir} && {_ros_setup} && source install/setup.bash && "
        f"ros2 launch unitree_lidar_ros2 launch.py start_rviz:=false "
        f"lidar_ip:={_lidar_ip} local_ip:={_local_ip}",
        name="LiDAR",
    )

    # 等待 LiDAR 初始化 + 话题就绪
    _log("[启动] 等待 10s 让 LiDAR 驱动就绪...")
    time.sleep(10)

    # 验证 LiDAR 话题（同 start_prerequisites）
    try:
        result = subprocess.run(
            ["bash", "-c", f"{_ros_setup} && ros2 topic list 2>/dev/null | grep -q unilidar/cloud"],
            timeout=3, capture_output=True,
        )
        if result.returncode == 0:
            _log("[启动] ✅ LiDAR 话题已就绪")
        else:
            _log("[启动] ⚠️ LiDAR 话题未检测到（但仍继续）")
    except Exception:
        _log("[启动] ⚠️ 话题检测失败（但仍继续）")

    # ── 串口桥 ──
    if os.path.exists(_serial_port):
        _log(f"[启动] 启动串口桥 ({_serial_port})...")
        _serial_logdir = _PROJECT / "logs" / "serial"
        _serial_logdir.mkdir(parents=True, exist_ok=True)
        _leg_csv = str(_serial_logdir / f"{time.strftime('%Y-%m-%d_%H%M%S')}_leg.csv")
        launch(
            f"cd {_nav2_dir} && {_ros_setup} && source install/setup.bash && "
            f"ros2 launch dog_nav2_bringup chassis_serial_bridge.launch.py "
            f"  serial_port:={_serial_port} baud_rate:=115200 "
            f"  cmd_vel_topic:=/cmd_vel send_rate_hz:=50.0 "
            f"  active_state:=1 idle_state:=0 "
            f"  leg_debug_csv_path:={_leg_csv} "
            f"  leg_debug_log_period_sec:=0.5",
            name="串口桥",
        )
    else:
        _log(f"[启动] {_serial_port} 不存在，跳过串口桥")

    _log("[启动] ✅ 前置节点启动完成")


# ============================================================
# SIGINT 处理器（同 auto_task.py）
# ============================================================
def _sigint_handler(sig, frame):
    print("\n[退出] 收到 Ctrl+C，正在关闭...")
    cleanup_all()
    rclpy.shutdown()


# ============================================================
# 入口
# ============================================================

def main():
    signal.signal(signal.SIGINT, _sigint_handler)
    os.environ.setdefault("ROS_LOG_DIR", str(_PROJECT / "logs" / "ros"))

    global FORWARD_SPEED, APPROACH_TIME, CARRY_TIME

    parser = argparse.ArgumentParser(description="开环自动控制脚本")
    parser.add_argument("--no-startup", action="store_true",
                        help="不启动前置节点（LiDAR/串口桥需已运行）")
    parser.add_argument("--vx", "--speed", type=float, default=FORWARD_SPEED,
                        help=f"前进速度 m/s (默认 {FORWARD_SPEED})")
    parser.add_argument("--approach", type=float, default=None,
                        help=f"第一次前进时长 s (默认 {APPROACH_TIME})")
    parser.add_argument("--carry", type=float, default=None,
                        help=f"第二次前进时长 s (默认 {CARRY_TIME})")
    args = parser.parse_args()

    # 通过全局变量透传 CLI 参数
    FORWARD_SPEED = args.vx
    if args.approach is not None:
        APPROACH_TIME = args.approach
    if args.carry is not None:
        CARRY_TIME = args.carry

    print("╔══════════════════════════════════════════════╗")
    print("║  开环自动控制                               ║")
    print("║  流程: 开门 → 前进 → 抓取 → 前进 → 放置    ║")
    print("╚══════════════════════════════════════════════╝")

    # ── 启动前置节点（LiDAR + 串口桥），不含 ICP/Nav2 ──
    if not args.no_startup:
        _startup()

    # ── ROS 初始化 ──
    rclpy.init()
    node = OpenLoopAutoNode()

    # ── 等待 DDS 发现完成 ──
    print(f"\n[启动] 等待 5s 让 ROS 节点就绪...")
    time.sleep(5)

    # ── 执行各步骤 ──
    try:
        node.step_open_gate()
        node.step_approach()

        t0 = time.monotonic()
        node.step_grab()
        t_grab = time.monotonic() - t0
        node.get_logger().info(f"[耗时] 抓取阶段用时 {t_grab:.1f}s")

        time.sleep(1.0)  # 短暂停让机械臂稳定

        node.step_carry()
        node.step_place()
        node.finish()

    except KeyboardInterrupt:
        node.get_logger().info("\n[中断] 收到 Ctrl+C")
        node.stop()
        node.send_auto_cmd(AUTO_CMD_FINISH)
        node._cleanup_subprocesses()
    except Exception as e:
        node.get_logger().error(f"[错误] {e}")
        import traceback
        node.get_logger().error(traceback.format_exc())
        node.stop()
        node._cleanup_subprocesses()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        cleanup_all()


if __name__ == "__main__":
    main()
