# Copyright (c) 2021 PickNik, Inc.
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
#
# Author: Denis Stogl

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    declared_arguments = []
    # UR specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            description="Type/series of used UR robot.",
            choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e"],
            default_value="ur5e",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            description="IP address by which the robot can be reached.",
            default_value="xxx.xxx.xxx.xxx",
        )
    )
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="ros2_control_demo_bringup",
            description='Package with the controller\'s configuration in "config" folder. \
        Usually the argument is not set, it enables use of a custom setup.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="admittance_demo_controllers.yaml",
            description="YAML file with the controllers configuration.",
        )
    )

    # Initialize Arguments
    robot_ip = LaunchConfiguration("robot_ip")
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")

    # load urdf
    urdf_file_name = 'ur5_robotiq.urdf'
    urdf = os.path.join(
        get_package_share_directory('ur_description'), 'urdf', urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    # node arguments
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("rrbot_description"), "admittance_demo", "admittance_demo.rviz"]
    )

    # parameters
    robot_description = {"robot_description": robot_desc}
    initial_joint_controllers = PathJoinSubstitution([FindPackageShare(runtime_config_package), "config", controllers_file])

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            initial_joint_controllers,
        ],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    dashboard_client_node = Node(
        package="ur_robot_driver",
        executable="dashboard_client",
        name="dashboard_client",
        output="screen",
        # emulate_tty=True,
        parameters=[{"robot_ip": robot_ip}],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    admittance_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["admittance_controller", "-c", "/controller_manager"],
    )

    trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
    )

    faked_forces_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        # condition=True,
        arguments=["faked_forces_controller", "-c", "/controller_manager"],
    )
    ft_frame_node =Node(
        package='tf2_ros',
        executable='static_transform_publisher',                             # ee_link
        arguments = ['--x', '0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'wrist_3_link', '--child-frame-id', 'ft_frame']
    )

    nodes_to_start = [
        control_node,
        dashboard_client_node,
        robot_state_publisher_node,
        rviz_node,
        trajectory_controller_spawner,
        admittance_controller_spawner,
        faked_forces_controller_spawner,
        ft_frame_node,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
