# 底盘串口协议桥接（cmd_vel → STM32）

## 协议（与你提供的 API 一致）

- **方向**：上位机 → STM32  
- **帧结构**：`[0x55][0xAA][FuncID][Len][Payload...][CheckSum]`  
- **FuncID**：`0x10`（底盘移动指令）  
- **Len**：`9`  
- **Payload**：9 字节（小端）  
  - `vx` (m/s) ← `Twist.linear.x`（4字节 float32）  
  - `wz` (rad/s) ← `Twist.angular.z`（4字节 float32）  
  - `state`（1字节 uint8）  
- **CheckSum**：`Byte0` 到 `Byte(4+N-1)` 的累加和，取低 8 位（`& 0xFF`）

**完整帧长度**：`2 + 1 + 1 + 9 + 1 = 14` 字节。

示例（vx=0.1, wz=0.0, state=1）：
`0x55 0xAA 0x10 0x09 0xCD 0xCC 0xCC 0x3D 0x00 0x00 0x00 0x00 0x01 0x2A`

## 100ms 看门狗

固件超过 100ms 收不到包会刹停。本节点默认 **50Hz** 定时发送；若一段时间没有新的 `cmd_vel`，会发 **全 0**。

- `active_state`：有新速度指令时发送的状态字节（默认 `1`）
- `idle_state`：超时/停车时发送的状态字节（默认 `0`）

## 依赖

```bash
sudo apt install python3-serial
```

## 编译与运行

```bash
cd ~/nav2_ws
colcon build --packages-select dog_nav2_bringup
source install/setup.bash
```

单独启动桥（Nav2 已运行时）：

```bash
ros2 launch dog_nav2_bringup chassis_serial_bridge.launch.py \
  serial_port:=/dev/ttyUSB0 \
  baud_rate:=115200 \
  cmd_vel_topic:=/cmd_vel \
  active_state:=1 \
  idle_state:=0
```

## 权限

串口权限不足时：

```bash
sudo usermod -aG dialout $USER
# 重新登录后生效，或临时 sudo chmod 666 /dev/ttyUSB0
```
