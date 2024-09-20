#!/usr/bin/env python3
#
# SPDX-License-Identifier: Apache-2.0
#
# Copyright (c) 2022 MangDang
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
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    ld = LaunchDescription()
    
    bringup_path = os.path.join(
        get_package_share_directory('mini_pupper_bringup'),
        'launch/bringup.launch.py'
    )

    launch_bringup=IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_path)
    )

    display_node = Node(
            package='mini_pupper_driver',
            executable='display_interface',
            name='display_interface',
            remappings=[
                ("mini_pupper_lcd/image_raw", "image_raw")
            ],
            output='screen'
        )

    ld.add_action(launch_bringup)
    ld.add_action(display_node)
    
    return ld
