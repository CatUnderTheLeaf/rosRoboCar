#!/usr/bin/env python

# Copyright 1996-2021 Cyberbotics Ltd.
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

"""Launch Webots Tesla driver."""

import os
import pathlib
import launch
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from webots_ros2_driver.webots_launcher import WebotsLauncher


def generate_launch_description():
    car_package_dir = get_package_share_directory('webots_ros2_tesla')
    package_dir = get_package_share_directory('robo_car')
    world = LaunchConfiguration('world')
    robot_description = pathlib.Path(os.path.join(car_package_dir, 'resource', 'tesla_webots.urdf')).read_text()
    ros2_control_params = os.path.join(package_dir, 'resource', 'ros2_control.yaml')

    webots = WebotsLauncher(
        world=PathJoinSubstitution([car_package_dir, 'worlds', world])
    )

    controller_manager_timeout = ['--controller-manager-timeout', '100']
    controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''

    use_deprecated_spawner_py = 'ROS_DISTRO' in os.environ and os.environ['ROS_DISTRO'] == 'foxy'

    trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner' if not use_deprecated_spawner_py else 'spawner.py',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['ur_joint_trajectory_controller'] + controller_manager_timeout,
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner' if not use_deprecated_spawner_py else 'spawner.py',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['ur_joint_state_broadcaster'] + controller_manager_timeout,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': '<robot name=""><link name=""/></robot>'
        }],
    )

    tesla_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        additional_env={'WEBOTS_ROBOT_NAME': 'vehicle'},
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True},
            # {'set_robot_state_publisher': True},
            # ros2_control_params
        ]
    )

    lane_follower = Node(
        package='webots_ros2_tesla',
        executable='lane_follower',
    )

    # check if path_from_image node works
    path_from_image = Node(
        package='path_from_image',
        executable='my_node',
        output='screen',
    )

    static_tf2_publisher = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['1.67', '0', '1.072', '0', '0', '0', 'base_link', 'camera']
      )
# camera
# translation: -2.12 0 0.93
# rotation 0 0 1 0
# scale 1 1 1

# [robot_state_publisher-4] Error:   Failed to build tree: parent link [slot_0] of joint [slot_0_rear left wheel_joint] not found.  This is not valid according to the URDF spec. Every link you refer to from a joint needs to be explicitly defined in the robot description. To fix this problem you can either remove this joint [slot_0_rear left wheel_joint] from your urdf file, or add "<link name="slot_0" />" to your urdf file.
# [robot_state_publisher-4]          at line 225 in /tmp/binarydeb/ros-galactic-urdfdom-2.3.5/urdf_parser/src/model.cpp
# [robot_state_publisher-4] Failed to parse robot description using: urdf_xml_parser/URDFXMLParser
# [robot_state_publisher-4] [WARN] [1654272971.381278700] [robot_state_publisher]: Unable to initialize urdf::model from robot description

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='tesla_world.wbt',
            description='Choose one of the world files from `/webots_ros2_tesla/worlds` directory'
        ),
        # joint_state_broadcaster_spawner,
        # trajectory_controller_spawner,
        # robot_state_publisher,        
        webots,
        tesla_driver,
        lane_follower,
        
        path_from_image,
        static_tf2_publisher,
        # launch.actions.RegisterEventHandler(
        #     launch.event_handlers.OnProcessStart(
        #         target_action=tesla_driver,
        #         on_start=[
        #             static_tf2_publisher
        #         ]
        #     )
        # ),
        # This action will kill all nodes once the Webots simulation has exited
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])
