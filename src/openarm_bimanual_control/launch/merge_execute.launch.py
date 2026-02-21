# Copyright 2025 OpenArm Bimanual Control
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
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_control = get_package_share_directory("openarm_bimanual_control")
    pkg_description = get_package_share_directory("openarm_description")
    pkg_moveit_config = get_package_share_directory("openarm_bimanual_moveit_config")

    params_file = os.path.join(pkg_control, "config", "bimanual_merge_params.yaml")

    # Robot description (URDF) - same as demo: bimanual, fake hardware, ros2_control
    xacro_path = os.path.join(pkg_description, "urdf", "robot", "v10.urdf.xacro")
    robot_description = xacro.process_file(
        xacro_path,
        mappings={
            "arm_type": "v10",
            "bimanual": "true",
            "use_fake_hardware": "true",
            "ros2_control": "true",
            "left_can_interface": "can1",
            "right_can_interface": "can0",
        },
    ).toprettyxml(indent="  ")

    # Robot description semantic (SRDF)
    srdf_path = os.path.join(pkg_moveit_config, "config", "openarm_bimanual.srdf")
    with open(srdf_path, "r") as f:
        robot_description_semantic = f.read()

    node_params = [
        params_file,
        {"robot_description": robot_description, "robot_description_semantic": robot_description_semantic},
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=params_file,
            description="Path to bimanual merge params YAML",
        ),
        Node(
            package="openarm_bimanual_control",
            executable="merge_execute_node",
            name="merge_execute_node",
            output="screen",
            parameters=node_params,
        ),
    ])
