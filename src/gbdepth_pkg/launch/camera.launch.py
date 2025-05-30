import os
import launch
import launch_ros

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

sensor_params = [
    {'sensor_dist': 697.0}, #[pix]
    {'eye_position': 0.735}, #[m]
    {'frame_id': 'base_link'},
    {'pixel_step': 4},
    {'pos_x': 0.0},
    {'pos_y': 0.0},
    {'pos_z': 0.735},
]


def generate_launch_description():

    ld = LaunchDescription()

    camera_node = Node(
        package='gbdepth_pkg',
        executable='camera_node',
        name = 'camera_node',
        remappings= [('camera_img', 'camera_img')],
        parameters=[
            {'camera_num':0}, 
            # {'camera_num':4}, 
            {'publish_duration':100} #[msec]
        ],
    )
    ld.add_action(camera_node)

    depth_estimation = Node(
        package='gbdepth_pkg',
        executable = 'depth_estimation_node',
        name = 'depth_estimation_node',
        parameters = sensor_params,
        remappings= [('camera_img', 'camera_img')],
    )
    ld.add_action(depth_estimation)

    ground_estimation = Node(
        package='gbdepth_pkg',
        executable = 'ground_estimation_node',
        name = 'ground_estimation_node',
    )
    ld.add_action(ground_estimation)

    return ld