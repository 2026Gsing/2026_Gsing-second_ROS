#!/usr/bin/env python3
"""
send_chassis_test_serial.py — 底盘串口通信测试工具（无需启动 Nav2）

功能：
  直接通过串口向 STM32 发送底盘控制帧，用于测试串口通信链路
  和底盘运动功能是否正常。

协议帧:
  [0x55][0xAA][0x10][0x09][vx(float32)][wz(float32)][state(uint8)][checksum]
  - vx: 线速度 (m/s)，正值=前进, 负值=后退
  - wz: 角速度 (rad/s)，正值=左转, 负值=右转
  - state: 机器人运动状态，默认从 vx/wz 自动推导：
      0=IDLE 1=FORWARD 2=BACKWARD 3=LEFT 4=RIGHT
  - checksum = sum(前面所有字节) & 0xFF

使用示例:
  # 前进 0.1m/s，持续 2 秒
  python3 send_chassis_test_serial.py --port /dev/ttyACM0 --vx 0.10 --wz 0.00 --rate 50 --duration 2 --send-stop-on-exit

  # 原地旋转（角速度主导，自动推导为 LEFT/RIGHT）
  python3 send_chassis_test_serial.py --port /dev/ttyACM0 --vx 0.00 --wz 0.50 --rate 50 --duration 3 --send-stop-on-exit

  # 后退（线速度主导，自动推导为 BACKWARD）
  python3 send_chassis_test_serial.py --port /dev/ttyACM0 --vx -0.10 --wz 0.00 --rate 50 --duration 3 --send-stop-on-exit
"""

import argparse
import struct
import sys
import time

try:
    import serial
except ImportError as e:
    print("缺少 pyserial，请先安装: sudo apt install python3-serial")
    raise e


# ============ 协议常量 ============
HEAD1 = 0x55
HEAD2 = 0xAA
FUNC_CHASSIS_MOVE = 0x10
PAYLOAD_LEN = 9
PACK_FMT = "<2fB"  # vx(float32), wz(float32), state(uint8)

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
    if abs(wz) > abs(vx) and abs(wz) > _STATE_EPSILON:
        return ROBOT_STATE_LEFT if wz >= 0.0 else ROBOT_STATE_RIGHT
    if abs(vx) > _STATE_EPSILON:
        return ROBOT_STATE_FORWARD if vx >= 0.0 else ROBOT_STATE_BACKWARD
    return ROBOT_STATE_IDLE


def build_packet(vx: float, wz: float, state: int) -> bytes:
    """组装串口协议帧"""
    payload = struct.pack(PACK_FMT, float(vx), float(wz), int(state) & 0xFF)
    frame_wo_checksum = bytes([HEAD1, HEAD2, FUNC_CHASSIS_MOVE, PAYLOAD_LEN]) + payload
    checksum = sum(frame_wo_checksum) & 0xFF
    return frame_wo_checksum + bytes([checksum])


def to_hex(data: bytes) -> str:
    """将字节流转为十六进制显示"""
    return " ".join(f"{b:02X}" for b in data)


def parse_args():
    parser = argparse.ArgumentParser(description="发送底盘串口测试帧（vx, wz, state）")
    parser.add_argument("--port", type=str, default="/dev/ttyUSB0", help="串口设备，例如 /dev/ttyUSB0 或 /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200, help="波特率")
    parser.add_argument("--vx", type=float, default=0.0, help="线速度 m/s（正值=前进, 负值=后退）")
    parser.add_argument("--wz", type=float, default=0.0, help="角速度 rad/s（正值=左转, 负值=右转）")
    parser.add_argument("--state", type=int, default=None,
                        help="手动指定状态字节 0-4（默认从 vx/wz 自动推导）")
    parser.add_argument("--rate", type=float, default=50.0, help="发送频率 Hz")
    parser.add_argument("--duration", type=float, default=2.0, help="发送时长（秒），<0 表示一直发")
    parser.add_argument("--print-every", type=int, default=20, help="每 N 帧打印一次十六进制，0 不打印")
    parser.add_argument("--send-stop-on-exit", action="store_true", help="退出前发送一帧停止包(vx=0, wz=0, state=0)")
    return parser.parse_args()


def main():
    args = parse_args()
    if args.rate <= 0:
        print("rate 必须 > 0")
        return 1

    # state 自动推导：未指定时从 vx/wz 计算（与 STM32 逻辑一致）
    if args.state is not None:
        state = int(args.state) & 0xFF
    else:
        state = derive_robot_state(args.vx, args.wz)

    period = 1.0 / args.rate
    pkt = build_packet(args.vx, args.wz, state)
    stop_pkt = build_packet(0.0, 0.0, ROBOT_STATE_IDLE)

    state_names = {0: "IDLE", 1: "FWD", 2: "REV", 3: "LEFT", 4: "RIGHT"}
    state_label = state_names.get(state, f"?{state}")
    print(f"打开串口 {args.port} @ {args.baud}")
    print(f"速度 vx={args.vx:.3f} wz={args.wz:.3f} → state={state}({state_label})")
    ser = serial.Serial(port=args.port, baudrate=args.baud, timeout=0.05)

    start = time.monotonic()
    count = 0
    print("开始发送，按 Ctrl+C 退出")
    try:
        while True:
            now = time.monotonic()
            if args.duration >= 0 and (now - start) > args.duration:
                break

            ser.write(pkt)
            ser.flush()
            count += 1

            if args.print_every > 0 and (count % args.print_every == 0):
                print(f"[{count}] {to_hex(pkt)}")

            time.sleep(period)
    except KeyboardInterrupt:
        print("\n收到 Ctrl+C，停止发送")
    finally:
        # 退出前发送停止包，确保底盘停止运动
        if args.send_stop_on_exit:
            try:
                ser.write(stop_pkt)
                ser.flush()
                print(f"[stop] {to_hex(stop_pkt)}")
            except Exception:
                pass
        ser.close()

    elapsed = time.monotonic() - start
    print(f"发送完成: {count} 帧, 耗时 {elapsed:.2f}s, 平均 {count / max(elapsed, 1e-6):.1f}Hz")
    return 0


if __name__ == "__main__":
    sys.exit(main())
