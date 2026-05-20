#!/usr/bin/env python3
import serial
import time

# 尝试自动检测串口
import serial.tools.list_ports
ports = serial.tools.list_ports.comports()
for port in ports:
    if '0483:5740' in port.hwid or 'STM32' in port.description:
        print(f"找到STM32串口: {port.device}")
        SERIAL_PORT = port.device
        break
else:
    SERIAL_PORT = '/dev/ttyACM0'
    print(f"使用默认串口: {SERIAL_PORT}")

print(f"正在监听 {SERIAL_PORT}...")
print("按 Ctrl+C 退出\n")

try:
    ser = serial.Serial(SERIAL_PORT, 115200, timeout=1)
    while True:
        if ser.in_waiting:
            data = ser.read(ser.in_waiting)
            print(data.hex(), end=' ')
            # 尝试显示ASCII
            try:
                print(data.decode('ascii', errors='ignore'), end='')
            except:
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
