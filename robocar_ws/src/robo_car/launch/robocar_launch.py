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

    lane_area_drawer = Node(
        package='path_from_image',
        executable='lane_area_drawer',
        output='screen',
        parameters=[            
            {'image_raw': '/vehicle/front_camera/image_raw',
            'lane_image': '/path/lane_image',
            'transf_matrix': '/path/transf_matrix',
            'img_waypoints': '/path/img_waypoints'}
        ]
    )

    trans_matrix_getter = Node(
        package='path_from_image',
        executable='trans_matrix_getter',
        output='screen',
        parameters=[
            {'_camera_frame': 'camera_link_optical',
            '_base_frame': 'chassis',
            'distance_ahead': 10.0,
            'lane_width': 10.0,
            'transf_matrix': '/path/transf_matrix',
            'camera_info': '/vehicle/front_camera/camera_info'}
        ]
    )

    path_publisher = Node(
        package='path_from_image',
        executable='path_publisher',
        output='screen',
        parameters=[
            {'_camera_frame': 'camera_link_optical',
            '_base_frame': 'chassis',
            'img_waypoints': '/path/img_waypoints',
            'path_waypoints': '/path/path_waypoints',
            'camera_info': '/vehicle/front_camera/camera_info'}
        ]
    )

    mock_camera = Node(
        package='robo_car',
        executable='mock_camera',
        output='screen',
        parameters=[
            {'camera_info': '/vehicle/front_camera/camera_info',
            'image_raw': '/vehicle/front_camera/image_raw',
            }
        ]
    )

    markers = Node(
        package='rviz_markers',
        executable='marker_publisher',
        output='screen',
        parameters=[
            {'path_waypoints': '/path/path_waypoints',
            'viz_waypoints': '/visualize/waypoints',
            }
        ]
    )



    return LaunchDescription([
        joint_state_publisher_node,
        robot_state_publisher,        
        
        mock_camera,

        trans_matrix_getter,
        lane_area_drawer,
        path_publisher,
        markers,

        # This action will kill all nodes once the Webots simulation has exited
        # launch.actions.RegisterEventHandler(
        #     event_handler=launch.event_handlers.OnProcessExit(
        #         target_action=path_from_image,
        #         on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
        #     )
        # )
    ])
