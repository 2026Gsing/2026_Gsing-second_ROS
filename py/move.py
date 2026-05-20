import serial
import serial.tools.list_ports
import time
import struct
import sys  # 用于接收命令行参数

# ===================== 【1】机器人通信协议定义 =====================
ROBOT_STATE_IDLE      = 0    # 待机
ROBOT_STATE_FORWARD   = 1    # 前进
ROBOT_STATE_BACKWARD  = 2    # 后退
ROBOT_STATE_LEFT      = 3    # 左转
ROBOT_STATE_RIGHT     = 4    # 右转
ROBOT_STATE_CROUCH    = 5    # 蹲下

# ===================== 【2】串口配置 =====================
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE   = 115200

# ===================== 【3】指令数据包构造 =====================
def build_move_packet(vx: float, wz: float, state: int):
    frame_header = [0xAA, 0x55]
    frame_tail = [0xBB]
    
    vx_bytes = struct.pack('<f', vx)
    wz_bytes = struct.pack('<f', wz)
    
    packet = bytes(frame_header) + vx_bytes + wz_bytes + bytes([state])
    checksum = sum(packet) & 0xFF
    packet += bytes([checksum]) + bytes(frame_tail)
    
    return packet

# ===================== 【4】发送指令函数 =====================
def send_robot_command(vx, wz, state):
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        if not ser.is_open:
            print("❌ 串口打开失败")
            return

        print(f"✅ 串口 {SERIAL_PORT} 已连接")
        packet = build_move_packet(vx, wz, state)
        ser.write(packet)

        state_str = {
            0: "待机", 1: "前进", 2: "后退", 3: "左转", 4: "右转", 5: "蹲下"
        }.get(state, "未知")
        
        print(f"📤 已发送指令: {state_str}")
        print(f"   速度 vx = {vx:.2f}  |  角速度 wz = {wz:.2f}")
        print(f"   数据包: {packet.hex()}\n")

        time.sleep(0.1)
        ser.close()
        print("🔌 串口已关闭")

    except Exception as e:
        print(f"❌ 发送失败: {e}")

# ===================== 【5】根据命令行参数执行指令 =====================
if __name__ == "__main__":
    # 获取命令行输入的数字 如 python move.py 1
    if len(sys.argv) < 2:
        print("用法: python move.py 0/1/2/3/4/5")
        print("0:待机  1:前进  2:后退  3:左转  4:右转  5:蹲下")
        sys.exit(1)

    cmd = int(sys.argv[1])

    # 根据数字执行对应指令
    if cmd == 1:
        send_robot_command(0.2, 0.0, ROBOT_STATE_FORWARD)
    elif cmd == 2:
        send_robot_command(-0.2, 0.0, ROBOT_STATE_BACKWARD)
    elif cmd == 3:
        send_robot_command(0.0, 0.2, ROBOT_STATE_LEFT)
    elif cmd == 4:
        send_robot_command(0.0, -0.2, ROBOT_STATE_RIGHT)
    elif cmd == 5:
        send_robot_command(0.0, 0.0, ROBOT_STATE_CROUCH)
    elif cmd == 0:
        send_robot_command(0.0, 0.0, ROBOT_STATE_IDLE)
    else:
        print("无效指令！")
