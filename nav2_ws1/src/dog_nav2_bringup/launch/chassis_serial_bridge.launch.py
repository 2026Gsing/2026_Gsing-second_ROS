#!/usr/bin/env python3

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
                description="Serial device for STM32",
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
                description="State byte when cmd_vel is fresh",
            ),
            DeclareLaunchArgument(
                "idle_state",
                default_value="0",
                description="State byte when command is stale/stop",
            ),
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
