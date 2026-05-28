#!/usr/bin/env python3
#
# Copyright 2025 ROBOTIS CO., LTD.
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
# Author: Woojin Wie

import os
import re
import subprocess
import time
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


# URDF <robot name="..."> → (config subfolder, srdf filename in that folder)
MODEL_DIRS = {
    'ffw_bg2_follower': ('ffw_common', 'ffw.srdf'),
    'ffw_sg2_follower': ('ffw_common', 'ffw.srdf'),
    'ffw_sh5_follower': ('ffw_sh5', 'ffw_sh5.srdf'),
    'ffw_bh5_follower': ('ffw_bh5', 'ffw_bh5.srdf'),
}

ALIASES = {
    'bg2': 'ffw_bg2_follower',
    'sg2': 'ffw_sg2_follower',
    'sh5': 'ffw_sh5_follower',
    'bh5': 'ffw_bh5_follower',
}


def _detect_robot_name(timeout=10.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            r = subprocess.run(
                ['ros2', 'param', 'get', '/robot_state_publisher', 'robot_description'],
                capture_output=True, text=True, timeout=3,
            )
            if r.returncode == 0:
                m = re.search(r'<robot[^>]*\sname="([^"]+)"', r.stdout)
                if m:
                    return m.group(1)
        except Exception:
            pass
        time.sleep(0.5)
    return None


def _resolve_model(model_arg):
    if model_arg == 'auto':
        name = _detect_robot_name()
        if name is None:
            raise RuntimeError(
                'model:=auto failed: /robot_state_publisher not reachable. '
                'Start Gazebo/bringup first, or pass model:=<bg2|sg2|sh5|bh5>.'
            )
    else:
        name = ALIASES.get(model_arg, model_arg)
    if name not in MODEL_DIRS:
        raise RuntimeError(
            f"Unsupported model '{name}'. Use bg2 / sg2 / sh5 / bh5."
        )
    return name


def _launch_setup(context, *_args, **_kwargs):
    start_rviz = LaunchConfiguration('start_rviz')
    use_sim = LaunchConfiguration('use_sim')
    warehouse_sqlite_path = LaunchConfiguration('warehouse_sqlite_path')
    publish_robot_description_semantic = LaunchConfiguration('publish_robot_description_semantic')

    robot_name = _resolve_model(LaunchConfiguration('model').perform(context))
    folder, srdf = MODEL_DIRS[robot_name]
    d = Path('config') / folder
    print(f'[moveit.launch.py] model: {robot_name} (config/{folder}/)')

    moveit_config = (
        MoveItConfigsBuilder(robot_name='ffw', package_name='ffw_moveit_config')
        .robot_description_semantic(str(d / srdf))
        .joint_limits(str(d / 'joint_limits.yaml'))
        .robot_description_kinematics(str(d / 'kinematics.yaml'))
        .trajectory_execution(str(d / 'moveit_controllers.yaml'))
        .to_moveit_configs()
    )

    warehouse_ros_config = {
        'warehouse_plugin': 'warehouse_ros_sqlite::DatabaseConnection',
        'warehouse_host': warehouse_sqlite_path,
    }

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            warehouse_ros_config,
            {
                'use_sim_time': use_sim,
                'publish_robot_description_semantic': publish_robot_description_semantic,
            },
        ],
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('ffw_moveit_config'), 'config', 'moveit.rviz']
    )
    rviz_node = Node(
        package='rviz2',
        condition=IfCondition(start_rviz),
        executable='rviz2',
        name='rviz2_moveit',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            warehouse_ros_config,
            {'use_sim_time': use_sim},
        ],
    )

    return [move_group_node, rviz_node]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'start_rviz', default_value='true', description='Whether to execute rviz2'
        ),
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            description='Whether to use simulation time',
        ),
        DeclareLaunchArgument(
            'model',
            default_value='auto',
            description=(
                "Robot model: 'auto' (detect from /robot_state_publisher) "
                'or bg2 / sg2 / sh5 / bh5'
            ),
        ),
        DeclareLaunchArgument(
            'warehouse_sqlite_path',
            default_value=os.path.expanduser('~/.ros/warehouse_ros.sqlite'),
            description='Path where the warehouse database should be stored',
        ),
        DeclareLaunchArgument(
            'publish_robot_description_semantic',
            default_value='true',
            description='Whether to publish robot description semantic',
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=_launch_setup)])
