#!/usr/bin/env python3
"""
chassis_serial_bridge.launch.py — Nav2 /cmd_vel → STM32 串口桥接启动

作用：
  启动 cmd_vel_chassis_serial 节点，将 Nav2 的速度命令通过串口协议
  转发给底盘下位机（STM32），实现运动控制。

串口协议帧格式：
  [0x55][0xAA][0x10][0x09][vx(float32)][wz(float32)][state(uint8)][checksum]

使用方式：
  ros2 launch dog_nav2_bringup chassis_serial_bridge.launch.py \
    serial_port:=/dev/ttyACM0 \
    baud_rate:=115200 \
    cmd_vel_topic:=/cmd_vel \
    send_rate_hz:=50.0 \
    active_state:=1 \
    idle_state:=0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "serial_port",
                default_value="/dev/ttyACM0",
                description="Serial device for STM32 (e.g. /dev/ttyACM0 or /dev/ttyUSB0)",
            ),
            DeclareLaunchArgument(
                "baud_rate",
                default_value="115200",
                description="Serial baud rate",
            ),
            DeclareLaunchArgument(
                "cmd_vel_topic",
                default_value="/cmd_vel",
                description="Nav2 velocity output topic",
            ),
            DeclareLaunchArgument(
                "send_rate_hz",
                default_value="50.0",
                description="Send rate (Hz), keep >=10 for 100ms watchdog",
            ),
            DeclareLaunchArgument(
                "active_state",
                default_value="1",
                description="State byte when cmd_vel is fresh (STM32 active control mode)",
            ),
            DeclareLaunchArgument(
                "idle_state",
                default_value="0",
                description="State byte when command is stale/stop (STM32 idle mode)",
            ),
            # ============ cmd_vel → STM32 串口转发节点 ============
            # 订阅 /cmd_vel，以固定频率通过串口发送底盘控制帧
            # 超过 stale_timeout 未收到新 cmd_vel 时自动发送 idle_state 停止包
            # 节点退出时自动发送停止帧以防机器人继续运动
            Node(
                package="dog_nav2_bringup",
                executable="cmd_vel_chassis_serial.py",
                name="cmd_vel_chassis_serial",
                output="screen",
                parameters=[
                    {"serial_port": LaunchConfiguration("serial_port")},
                    {
                        "baud_rate": ParameterValue(
                            LaunchConfiguration("baud_rate"), value_type=int
                        )
                    },
                    {"cmd_vel_topic": LaunchConfiguration("cmd_vel_topic")},
                    {
                        "send_rate_hz": ParameterValue(
                            LaunchConfiguration("send_rate_hz"), value_type=float
                        )
                    },
                    {
                        "active_state": ParameterValue(
                            LaunchConfiguration("active_state"), value_type=int
                        )
                    },
                    {
                        "idle_state": ParameterValue(
                            LaunchConfiguration("idle_state"), value_type=int
                        )
                    },
                ],
            ),
        ]
    )
