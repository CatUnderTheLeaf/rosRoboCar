#!/usr/bin/env python

import os
import pathlib
# import launch
# from launch.substitutions import LaunchConfiguration
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

# from launch.conditions import IfCondition


def generate_launch_description():
    package_dir = get_package_share_directory('robo_car')
    robot_description_file = pathlib.Path(os.path.join(package_dir, 'model', 'car.urdf'))
    robot_description = robot_description_file.read_text()
  
    robot_state_publisher = Node(
        package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': False, 'robot_description': robot_description}]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher'
    )

    # check if path_from_image node works
    path_from_image = Node(
        package='path_from_image',
        executable='image_warper',
        output='screen',
        parameters=[
            {'_camera_frame': 'camera_link_optical',
            '_base_frame': 'chassis',
            'distance_ahead': 10.0,
            'lane_width': 10.0}
        ]
    )

    static_tf2_publisher = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['1.67', '0', '1.072', '0', '0', '0', 'base_link', 'camera']
      )


    return LaunchDescription([
        joint_state_publisher_node,
        robot_state_publisher,        
        
        path_from_image,
        # static_tf2_publisher,
        
        # This action will kill all nodes once the Webots simulation has exited
        # launch.actions.RegisterEventHandler(
        #     event_handler=launch.event_handlers.OnProcessExit(
        #         target_action=path_from_image,
        #         on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
        #     )
        # )
    ])
