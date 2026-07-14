#!/usr/bin/python3
"""
detect_hardware.py — 自动扫描 LiDAR 网络和 STM32 串口

作爲模块被其他脚本调用:
  from py.tools.detect_hardware import detect_all
  hw = detect_all()  # → {"serial_port": ..., "lidar_iface": ..., "lidar_reachable": bool}

独立运行:
  ./ros-run.sh py/tools/detect_hardware.py
"""

import fnmatch
import os
import platform

try:
    import serial
    HAVE_SERIAL = True
except ImportError:
    HAVE_SERIAL = False
import socket
import subprocess
import sys
import time
from pathlib import Path


# ============================================================
# 工具
# ============================================================
def _run(cmd, timeout=3):
    """运行命令，返回 (rc, stdout, stderr)"""
    try:
        r = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=timeout)
        return r.returncode, r.stdout.strip(), r.stderr.strip()
    except subprocess.TimeoutExpired:
        return -1, "", "timeout"
    except Exception as e:
        return -1, "", str(e)


def _green(s):
    return f"\033[92m{s}\033[0m"


def _red(s):
    return f"\033[91m{s}\033[0m"


def _yellow(s):
    return f"\033[93m{s}\033[0m"


def _bold(s):
    return f"\033[1m{s}\033[0m"


# ============================================================
# LiDAR 网络检测（静默模式，返回 dict）
# ============================================================
LIDAR_KNOWN_IPS = ["192.168.1.62", "192.168.1.1"]
LIDAR_LOCAL_IP = "192.168.1.2"
LIDAR_PORT = 6101


def detect_lidar(verbose=True):
    """
    检测 LiDAR 网络连接。

    Args:
        verbose: 是否打印详细信息

    Returns:
        dict:
          iface       str/None  检测到的 LiDAR 网卡名
          local_ip    str/None  本机 LiDAR 网卡 IP
          lidar_ip    str/None  可达的 LiDAR IP（第一个 ping 通的）
          reachable   bool      LiDAR 是否可达
    """
    result = {"iface": None, "local_ip": None, "lidar_ip": None, "reachable": False}

    if verbose:
        print()
        print(_bold("═" * 50))
        print(_bold("  LiDAR 网络扫描"))
        print(_bold("═" * 50))

    # 1. 找物理网卡
    rc, out, _ = _run("ip -o link show | grep -v LOOPBACK | grep -v docker | grep -v vbox | awk -F': ' '{print $2}'")
    ifaces = [l.strip() for l in out.split("\n") if l.strip()] if out else []

    if not ifaces:
        if verbose:
            print(f"  {_red('✗')} 未检测到网卡")
        return result

    if verbose:
        print(f"  {_green('✓')} 检测到网卡: {', '.join(ifaces)}")

    # 2. 找 192.168.1.x 网段接口，优先有线
    candidates = []
    for iface in ifaces:
        rc, out, _ = _run(f"ip addr show {iface} 2>/dev/null | grep 'inet '")
        if out:
            ip = out.split()[1]  # e.g. "192.168.1.2/24"
            if verbose:
                print(f"  {_green('✓')} {iface}: {ip}", end="")
            if ip.startswith("192.168.1."):
                if verbose:
                    print(f"  ← {_yellow('LiDAR 网段')}")
                candidates.append((iface, ip))
            elif verbose:
                print()

    if not candidates:
        if verbose:
            print(f"  {_yellow('!')} 未找到 192.168.1.x 网段的接口")
        return result

    # 选择：优先有线(enp/eth)
    lidar_iface = None
    lidar_ip = None
    for pref in ["enp", "eth"]:
        for iface, ip in candidates:
            if iface.startswith(pref):
                lidar_iface = iface
                lidar_ip = ip
                break
        if lidar_iface:
            break
    if not lidar_iface:
        lidar_iface, lidar_ip = candidates[0]  # fallback 第一个

    result["iface"] = lidar_iface
    result["local_ip"] = lidar_ip.split("/")[0] if lidar_ip else None

    # 3. ping LiDAR IP
    if verbose:
        print()
    for lip in LIDAR_KNOWN_IPS:
        rc, _, _ = _run(f"ping -c 1 -W 1 {lip}")
        if rc == 0:
            result["lidar_ip"] = lip
            result["reachable"] = True
            if verbose:
                print(f"  {_green('✓')} LiDAR {lip} → 可达")
            break
        elif verbose:
            print(f"  {_red('✗')} LiDAR {lip} → 不可达")

    # 4. UDP 端口测试
    if verbose and lidar_iface:
        print()
        print(f"  尝试连接 LiDAR UDP 端口 {LIDAR_PORT}...")
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
            sock.settimeout(1.0)
            sock.bind(("0.0.0.0", 6201))
            sock.sendto(b"", (LIDAR_KNOWN_IPS[0], LIDAR_PORT))
            data, addr = sock.recvfrom(1024)
            print(f"  {_green('✓')} 收到 LiDAR 数据包: {len(data)} bytes 来自 {addr[0]}:{addr[1]}")
            sock.close()
        except socket.timeout:
            print(f"  {_red('✗')} 未收到 LiDAR 响应（超时）")
        except OSError as e:
            print(f"  {_yellow('!')} UDP 绑定失败: {e}")
        except Exception as e:
            print(f"  {_yellow('!')} UDP 异常: {e}")

    return result


# ============================================================
# 串口检测（静默模式，返回端口路径）
# ============================================================
def detect_serial(verbose=True):
    """
    扫描并识别 STM32 串口。

    Args:
        verbose: 是否打印详细信息

    Returns:
        str/None: STM32 串口设备路径（如 /dev/ttyACM0），未找到返回 None
    """
    if verbose:
        print()
        print(_bold("═" * 50))
        print(_bold("  STM32 串口扫描"))
        print(_bold("═" * 50))

    # 1. 列出候选设备
    candidates = []
    for pattern in ["ttyACM*", "ttyUSB*"]:
        try:
            for f in os.listdir("/dev"):
                if fnmatch.fnmatch(f, pattern):
                    candidates.append(f"/dev/{f}")
        except OSError:
            pass

    if not candidates:
        if verbose:
            print(f"  {_red('✗')} 未检测到串口设备")
        return None

    # 2. 无 pyserial 时只能检查存在性
    if not HAVE_SERIAL:
        if verbose:
            print(f"  {_yellow('!')} pyserial 未安装（sudo apt install python3-serial），仅能检查存在性")
            for dev in sorted(candidates):
                try:
                    st = os.stat(dev)
                    perm = _green("可读") if (st.st_mode & 0o444) else _red("权限不足")
                    print(f"  ? {dev}  [{perm}]")
                except OSError:
                    print(f"  {_red('✗')} {dev}  [无权限]")
        return candidates[0] if candidates else None

    # 3. 逐个尝试打开，寻找 STM32（0x55 0xAA 帧头）
    found = None
    for dev in sorted(candidates):
        try:
            st = os.stat(dev)
            readable = bool(st.st_mode & 0o444)
        except OSError:
            readable = False

        perm_str = _green("可读") if readable else _red("权限不足")
        is_stm32 = False
        detail = ""

        try:
            ser = serial.Serial(dev, baudrate=115200, timeout=0.3)
            data = ser.read(32)
            ser.close()
            if data:
                hex_preview = " ".join(f"{b:02x}" for b in data[:16])
                if data[0] == 0x55 and len(data) > 1:
                    is_stm32 = True
                    detail = _green("← 匹配 STM32 协议帧")
                else:
                    detail = _yellow(f"← {hex_preview}")
            else:
                detail = _yellow("← 无数据")
        except (serial.SerialException, OSError) as e:
            detail = _yellow(str(e)[:50])
        except Exception as e:
            detail = _yellow(str(e)[:50])

        if is_stm32:
            found = dev
            if verbose:
                print(f"  {_green('✓')} {dev}  [{perm_str}] {detail}")
        elif verbose:
            marker = _green("✓") if is_stm32 else (_yellow("?") if detail else _red("✗"))
            print(f"  {marker} {dev}  [{perm_str}] {detail}")

    if verbose and not found:
        if candidates:
            # 没匹配到 STM32 帧头，但设备存在，返回第一个
            print(f"  {_yellow('!')} 未识别到 STM32 协议帧，使用第一个设备 {candidates[0]}")
        return candidates[0]  # fallback: 第一个串口
    return found


# ============================================================
# 统一检测入口（供外部调用）
# ============================================================
def detect_all(verbose=False):
    """
    检测所有硬件，返回结果 dict。

    Args:
        verbose: 是否打印详细信息

    Returns:
        dict: {serial_port, lidar_iface, lidar_local_ip, lidar_ip, lidar_reachable}
    """
    lidar = detect_lidar(verbose=verbose)
    serial_port = detect_serial(verbose=verbose)

    return {
        "serial_port": serial_port,
        "lidar_iface": lidar["iface"],
        "lidar_local_ip": lidar["local_ip"],
        "lidar_ip": lidar["lidar_ip"],
        "lidar_reachable": lidar["reachable"],
    }


# ============================================================
# ROS2 环境检查（仅打印）
# ============================================================
def check_ros_env():
    print()
    print(_bold("═" * 50))
    print(_bold("  ROS2 环境检查"))
    print(_bold("═" * 50))

    if os.path.isdir("/opt/ros/jazzy"):
        print(f"  {_green('✓')} ROS2 Jazzy 已安装")
    else:
        print(f"  {_red('✗')} ROS2 Jazzy 未安装在 /opt/ros/jazzy")

    rmw = os.environ.get("RMW_IMPLEMENTATION", "")
    if rmw == "rmw_cyclonedds_cpp":
        print(f"  {_green('✓')} RMW_IMPLEMENTATION={rmw}")
    else:
        print(f"  {_yellow('!')} RMW_IMPLEMENTATION={rmw or '(未设置)'}  ← 应为 rmw_cyclonedds_cpp")

    rc, out, _ = _run("dpkg -l ros-jazzy-rmw-cyclonedds-cpp 2>/dev/null | grep -c '^ii'")
    if out.strip() == "1":
        print(f"  {_green('✓')} ros-jazzy-rmw-cyclonedds-cpp 已安装")
    else:
        print(f"  {_red('✗')} ros-jazzy-rmw-cyclonedds-cpp 未安装")

    HERE = Path(__file__).resolve().parent.parent
    for ws in ["fastlio2_v2", "nav2_ws1"]:
        setup = HERE / ws / "install" / "setup.bash"
        if setup.exists():
            print(f"  {_green('✓')} {ws}/install/setup.bash 存在")
        else:
            print(f"  {_yellow('!')} {ws}/install/setup.bash 不存在 → 若已编译过可忽略")


# ============================================================
# 建议
# ============================================================
def print_recommendations(hw):
    print()
    print(_bold("═" * 50))
    print(_bold("  启动建议"))
    print(_bold("═" * 50))

    if hw["lidar_iface"]:
        print(f"\n  LiDAR 网卡: {hw['lidar_iface']}")
        print(f"  配置命令: sudo nmcli device set {hw['lidar_iface']} managed no && "
              f"sudo ip addr add 192.168.1.2/24 dev {hw['lidar_iface']}")
    else:
        print(f"\n  {_yellow('!')} 未检测到 LiDAR 网卡，请先连接 LiDAR 网线")

    if hw["serial_port"]:
        print(f"\n  STM32 串口: {hw['serial_port']}")
        print(f"  权限命令: sudo chmod 666 {hw['serial_port']}")
    else:
        print(f"\n  {_yellow('!')} 未检测到 STM32 串口，请先连接 STM32 USB")

    print(f"\n  启动命令: ./ros-run.sh py/control/box_pick_node.py")


# ============================================================
# CLI 入口
# ============================================================
def main():
    print(_bold("╔════════════════════════════════════════╗"))
    print(_bold("║      硬件环境检测工具                    ║"))
    print(_bold("╚════════════════════════════════════════╝"))
    print(f"  机器: {platform.node()}  ({platform.system()} {platform.release()})")
    print(f"  路径: {Path(__file__).resolve().parent.parent}")

    hw = detect_all(verbose=True)
    check_ros_env()
    print_recommendations(hw)

    print()
    print(_bold("═" * 50))
    print("  检测完成")
    print(_bold("═" * 50))
    print()


if __name__ == "__main__":
    main()
