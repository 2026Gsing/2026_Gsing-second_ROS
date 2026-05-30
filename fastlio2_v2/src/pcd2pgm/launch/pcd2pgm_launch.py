# Copyright 2025 Lihan Chen
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration

MAP_DIR = "/home/hyper/program/2026_Gsing-second_ROS/fastlio2_v2/src/unilidar_fastlio_ros2-ros2/PCD"


def generate_launch_description():
    bringup_dir = get_package_share_directory("pcd2pgm")
    params_file = LaunchConfiguration("params_file")

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(bringup_dir, "config", "pcd2pgm.yaml"),
        description="Full path to the ROS2 parameters file",
    )

    start_pcd2pgm_cmd = Node(
        package="pcd2pgm",
        executable="pcd2pgm_node",
        name="pcd2pgm",
        output="screen",
        parameters=[params_file],
    )

    # 等 pcd2pgm 发布地图后保存，保存完毕自动退出
    save_map_cmd = ExecuteProcess(
        cmd=[
            "bash", "-c",
            f"sleep 2 && "
            f"ros2 run nav2_map_server map_saver_cli "
            f"-t /map --occ map_topic:=/map "
            f"-f {MAP_DIR}/pgm_map "
            f"--ros-args -p map_subscribe_transient_local:=true && "
            f"echo '地图已保存到 {MAP_DIR}/pgm_map.yaml'"
        ],
        output="screen",
    )

    # save_map 退出后关闭整个 launch
    shutdown_on_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=save_map_cmd,
            on_exit=[EmitEvent(event=Shutdown())],
        )
    )

    ld = LaunchDescription()
    ld.add_action(SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"))
    ld.add_action(SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"))
    ld.add_action(declare_params_file_cmd)
    ld.add_action(start_pcd2pgm_cmd)
    ld.add_action(save_map_cmd)
    ld.add_action(shutdown_on_exit)

    return ld
