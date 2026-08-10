# Copyright 2026 WSG50 ROS maintainers
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("gripper_ip", default_value="192.168.1.160"),
        DeclareLaunchArgument("port", default_value="1501"),
        DeclareLaunchArgument("default_speed", default_value="0.01"),
        DeclareLaunchArgument("default_acceleration", default_value="1.0"),
        DeclareLaunchArgument("default_grasp_force", default_value="40.0"),
        DeclareLaunchArgument("goal_tolerance", default_value="0.001"),
        DeclareLaunchArgument("state_publish_rate", default_value="20.0"),
        DeclareLaunchArgument("auto_acknowledge_faults", default_value="false"),
        DeclareLaunchArgument("auto_home", default_value="false"),
        DeclareLaunchArgument("connect_timeout_ms", default_value="2000"),
        DeclareLaunchArgument("response_timeout_ms", default_value="2000"),
        DeclareLaunchArgument("motion_timeout_ms", default_value="30000"),
        DeclareLaunchArgument("reconnect_interval_ms", default_value="1000"),
        DeclareLaunchArgument("state_update_period_ms", default_value="50"),
    ]

    parameters = {
        "gripper_ip": LaunchConfiguration("gripper_ip"),
        "port": ParameterValue(LaunchConfiguration("port"), value_type=int),
        "default_speed": ParameterValue(
            LaunchConfiguration("default_speed"), value_type=float
        ),
        "default_acceleration": ParameterValue(
            LaunchConfiguration("default_acceleration"), value_type=float
        ),
        "default_grasp_force": ParameterValue(
            LaunchConfiguration("default_grasp_force"), value_type=float
        ),
        "goal_tolerance": ParameterValue(
            LaunchConfiguration("goal_tolerance"), value_type=float
        ),
        "state_publish_rate": ParameterValue(
            LaunchConfiguration("state_publish_rate"), value_type=float
        ),
        "auto_acknowledge_faults": ParameterValue(
            LaunchConfiguration("auto_acknowledge_faults"), value_type=bool
        ),
        "auto_home": ParameterValue(
            LaunchConfiguration("auto_home"), value_type=bool
        ),
        "connect_timeout_ms": ParameterValue(
            LaunchConfiguration("connect_timeout_ms"), value_type=int
        ),
        "response_timeout_ms": ParameterValue(
            LaunchConfiguration("response_timeout_ms"), value_type=int
        ),
        "motion_timeout_ms": ParameterValue(
            LaunchConfiguration("motion_timeout_ms"), value_type=int
        ),
        "reconnect_interval_ms": ParameterValue(
            LaunchConfiguration("reconnect_interval_ms"), value_type=int
        ),
        "state_update_period_ms": ParameterValue(
            LaunchConfiguration("state_update_period_ms"), value_type=int
        ),
    }

    node = Node(
        package="wsg50_driver",
        executable="wsg50_gripper_driver_node",
        name="wsg50_gripper_driver",
        output="screen",
        parameters=[parameters],
    )
    return LaunchDescription(arguments + [node])
