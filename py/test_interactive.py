#!/usr/bin/env python3
"""
@brief 机械臂通信协议测试工具 (已适配 C 固件 3轴 XYZ 格式)
@usage python3 arm_protocol_test.py
依赖: pip install pyserial
功能: 通过串口发送自定义协议指令，控制机械臂XYZ坐标、吸盘、底盘、步态
"""

# 导入依赖库
import serial
import serial.tools.list_ports  # 提前导入，避免作用域问题
import struct
import time
import sys

# ==================== 硬件通信配置 ====================
SERIAL_PORT = '/dev/ttyACM0'  # 使用 ttyACM0
BAUD_RATE = 115200            # 波特率 必须与单片机一致

# ==================== 协议帧定义 (与C语言固件完全匹配) ====================
HEAD1 = 0x55                  # 帧头固定字节1
HEAD2 = 0xAA                  # 帧头固定字节2

FUNC_CHASSIS        = 0x10    # 功能码：底盘运动
FUNC_GAIT           = 0x11    # 功能码：步态切换
FUNC_ARM_CONTROL    = 0x12    # 功能码：机械臂控制
FUNC_SUCTION_CONTROL= 0x13    # 功能码：吸盘开关

# ==================== 核心协议打包函数 ====================

def build_frame(func_id, payload):
    """
    功能：构建完整协议帧
    帧格式：帧头1 + 帧头2 + 功能码 + 数据长度 + 数据载荷 + 校验和
    参数：
        func_id   功能码
        payload   数据段（bytes类型）
    返回：完整的一帧数据（bytes）
    """
    # 第1部分：拼接帧头 + 功能码 + 数据长度（共4字节）
    frame = bytes([HEAD1, HEAD2, func_id, len(payload)])
    
    # 第2部分：加上实际数据
    frame += payload
    
    # 第3部分：计算校验和（前面所有字节相加，取低8位）
    checksum = sum(frame) & 0xFF
    
    # 第4部分：把校验和放入帧尾
    frame += bytes([checksum])
    
    # 返回完整帧
    return frame

def float_to_le(value):
    """
    功能：将浮点数 转为 小端模式4字节bytes
    单片机通信标准格式
    """
    return struct.pack('<f', value)

def uint8_to_bytes(value):
    """
    功能：将0~255的单字节数字 转为 bytes
    """
    return bytes([value])

# ==================== 上层指令封装函数 ====================

def cmd_arm_control(x, y, z):
    """
    机械臂XYZ坐标控制 (0x12)
    适配C固件：3个float = 12字节
    """
    payload = float_to_le(x)
    payload += float_to_le(y)
    payload += float_to_le(z)
    return build_frame(FUNC_ARM_CONTROL, payload)

def cmd_suction(on):
    """
    吸盘控制 (0x13)
    参数：on=True开启，False关闭
    """
    payload = uint8_to_bytes(1 if on else 0)
    return build_frame(FUNC_SUCTION_CONTROL, payload)

def cmd_chassis(vx, vy, wz):
    """
    底盘运动控制 (0x10)
    vx:前后速度 vy:左右速度 wz:旋转速度
    """
    payload = float_to_le(vx)
    payload += float_to_le(vy)
    payload += float_to_le(wz)
    return build_frame(FUNC_CHASSIS, payload)

def cmd_gait(gait_id):
    """
    步态切换指令 (0x11)
    """
    payload = uint8_to_bytes(gait_id)
    return build_frame(FUNC_GAIT, payload)

# ==================== 主程序入口 ====================

def main():
    # 打印标题
    print("=" * 50)
    print("机械臂通信协议测试工具 (已适配 C 固件 XYZ 格式)")
    print("=" * 50)
    
    # 打开串口
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"✓ 已连接串口: {SERIAL_PORT}")
    except Exception as e:
        print(f"✗ 串口打开失败: {e}")
        print("\n可用串口列表:")
        # 直接使用已经导入的 serial.tools.list_ports
        for port in serial.tools.list_ports.comports():
            print(f"  - {port.device}: {port.description}")
        return
    
    # 等待设备稳定
    time.sleep(0.5)
    ser.reset_input_buffer()  # 清空接收缓存
    
    # 打印指令菜单
    print("\n" + "=" * 50)
    print("测试命令菜单:")
    print("=" * 50)
    print("1. 机械臂复位 (XYZ=0,0,0)")
    print("2. 机械臂移动到位置A (抬起)")
    print("3. 机械臂移动到位置B (下降)")
    print("4. 吸盘开启")
    print("5. 吸盘关闭")
    print("6. 自定义 XYZ 坐标")
    print("7. 连续循环测试")
    print("q. 退出")
    print("=" * 50)
    
    # 循环接收用户指令
    while True:
        try:
            choice = input("\n请选择命令 (1-7, q): ").strip()
            
            if choice == 'q':
                break  # 退出循环
            
            elif choice == '1':
                # 机械臂复位 XYZ=0,0,0
                print("→ 发送: 机械臂复位 XYZ=0,0,0")
                frame = cmd_arm_control(0, 0, 0)
                ser.write(frame)
                print(f"  发送字节: {frame.hex()}")
                
            elif choice == '2':
                # 移动到抬起位置A
                print("→ 发送: 移动到位置A (抬起)")
                x, y, z = 0.1, 0.0, 0.15
                frame = cmd_arm_control(x, y, z)
                ser.write(frame)
                print(f"  XYZ: {x:.2f}, {y:.2f}, {z:.2f}")
                print(f"  发送字节: {frame.hex()}")
                
            elif choice == '3':
                # 移动到下降位置B
                print("→ 发送: 移动到位置B (下降)")
                x, y, z = 0.15, 0.0, 0.05
                frame = cmd_arm_control(x, y, z)
                ser.write(frame)
                print(f"  XYZ: {x:.2f}, {y:.2f}, {z:.2f}")
                print(f"  发送字节: {frame.hex()}")
                
            elif choice == '4':
                # 吸盘开启
                print("→ 发送: 吸盘开启")
                frame = cmd_suction(True)
                ser.write(frame)
                print(f"  发送字节: {frame.hex()}")
                
            elif choice == '5':
                # 吸盘关闭
                print("→ 发送: 吸盘关闭")
                frame = cmd_suction(False)
                ser.write(frame)
                print(f"  发送字节: {frame.hex()}")
                
            elif choice == '6':
                # 自定义XYZ坐标控制
                try:
                    x = float(input("  X 坐标: "))
                    y = float(input("  Y 坐标: "))
                    z = float(input("  Z 坐标: "))
                    
                    print(f"→ 发送: XYZ={x:.2f}, {y:.2f}, {z:.2f}")
                    frame = cmd_arm_control(x, y, z)
                    ser.write(frame)
                    print(f"  发送字节: {frame.hex()}")
                except ValueError:
                    print("  ✗ 输入无效，请输入数字")
                    
            elif choice == '7':
                # 循环测试：位置A ↔ 位置B 自动切换
                print("→ 开始连续循环测试 (Ctrl+C 停止)")
                try:
                    while True:
                        # 发送位置A
                        frame = cmd_arm_control(0.10, 0.0, 0.15)
                        ser.write(frame)
                        time.sleep(1)
                        
                        # 发送位置B
                        frame = cmd_arm_control(0.15, 0.0, 0.05)
                        ser.write(frame)
                        time.sleep(1)
                        
                except KeyboardInterrupt:
                    print("\n  测试停止")
                    
            else:
                print("  无效选择，请重新输入")
                
        # 捕获 Ctrl+C 退出
        except KeyboardInterrupt:
            print("\n\n退出程序")
            break
        
        # 捕获串口通信错误
        except Exception as e:
            print(f"  ✗ 通信错误: {e}")
            break
    
    # 关闭串口
    ser.close()
    print("\n✓ 程序结束")

# 运行主函数
if __name__ == '__main__':
    main()