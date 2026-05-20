#!/usr/bin/env python3
"""
按下位机协议发送底盘测试帧（无需启动 Nav2）。

协议帧:
  [0x55][0xAA][0x10][0x09][vx(float32)][wz(float32)][state(uint8)][checksum]
  checksum = sum(前面所有字节) & 0xFF
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


HEAD1 = 0x55
HEAD2 = 0xAA
FUNC_CHASSIS_MOVE = 0x10
PAYLOAD_LEN = 9
PACK_FMT = "<2fB"  # vx(float32), wz(float32), state(uint8)


def build_packet(vx: float, wz: float, state: int) -> bytes:
    payload = struct.pack(PACK_FMT, float(vx), float(wz), int(state) & 0xFF)
    frame_wo_checksum = bytes([HEAD1, HEAD2, FUNC_CHASSIS_MOVE, PAYLOAD_LEN]) + payload
    checksum = sum(frame_wo_checksum) & 0xFF
    return frame_wo_checksum + bytes([checksum])


def to_hex(data: bytes) -> str:
    return " ".join(f"{b:02X}" for b in data)


def parse_args():
    parser = argparse.ArgumentParser(description="发送底盘串口测试帧（vx,wz,state）")
    parser.add_argument("--port", type=str, default="/dev/ttyUSB0", help="串口设备，例如 /dev/ttyUSB0 或 /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200, help="波特率")
    parser.add_argument("--vx", type=float, default=0.0, help="线速度 m/s")
    parser.add_argument("--wz", type=float, default=0.0, help="角速度 rad/s")
    parser.add_argument("--state", type=int, default=1, help="状态字节 0~255")
    parser.add_argument("--rate", type=float, default=50.0, help="发送频率 Hz")
    parser.add_argument("--duration", type=float, default=2.0, help="发送时长（秒），<0 表示一直发")
    parser.add_argument("--print-every", type=int, default=20, help="每 N 帧打印一次十六进制，0 不打印")
    parser.add_argument("--send-stop-on-exit", action="store_true", help="退出前发送一帧停止包(vx=0,wz=0,state=0)")
    return parser.parse_args()


def main():
    args = parse_args()
    if args.rate <= 0:
        print("rate 必须 > 0")
        return 1

    period = 1.0 / args.rate
    pkt = build_packet(args.vx, args.wz, args.state)
    stop_pkt = build_packet(0.0, 0.0, 0)

    print(f"打开串口 {args.port} @ {args.baud}")
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
