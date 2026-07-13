#!/usr/bin/env python3
"""
listen_serial.py — STM32 串口监听工具

功能：
  自动检测 STM32 串口设备，实时显示串口接收到的十六进制数据。
  用于调试底盘/机械臂的串口通信，确认下位机返回数据是否正确。

使用方式：
  python3 listen_serial.py
  （按 Ctrl+C 退出）

自动检测逻辑：
  1. 遍历系统串口设备
  2. 匹配 VID:PID = 0483:5740（STM32 的 USB 设备 ID）
  3. 匹配 description 包含 "STM32"
  4. 均未匹配则使用默认 /dev/ttyACM0
"""

import serial
import time

import serial.tools.list_ports

# ==================== 自动检测 STM32 串口 ====================
ports = serial.tools.list_ports.comports()
SERIAL_PORT = '/dev/ttyACM0'
for port in ports:
    if '0483:5740' in port.hwid or 'STM32' in port.description:
        print(f"找到STM32串口: {port.device}")
        SERIAL_PORT = port.device
        break
else:
    print(f"使用默认串口: {SERIAL_PORT}")

# ==================== 主循环：实时显示串口数据 ====================
print(f"正在监听 {SERIAL_PORT}...")
print("按 Ctrl+C 退出\n")

try:
    ser = serial.Serial(SERIAL_PORT, 115200, timeout=1)
    while True:
        if ser.in_waiting:
            data = ser.read(ser.in_waiting)
            # 十六进制显示
            print(data.hex(), end=' ')
            # 尝试 ASCII 显示（忽略非 ASCII 字符）
            try:
                print(data.decode('ascii', errors='ignore'), end='')
            except Exception:
                pass
            print()
        time.sleep(0.01)
except KeyboardInterrupt:
    print("\n\n停止监听")
except Exception as e:
    print(f"错误: {e}")
finally:
    if 'ser' in locals():
        ser.close()
