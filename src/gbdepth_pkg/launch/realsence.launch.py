import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

# pixel-pitch => 2.8[um/pix]
# sensor-distance => 4.38[mm]
# sensor-distanse => 1564.285...[pix]

# # for realsense on desk
# sensor_params = [
#     {'sensor_dist':364.8}, #[pix]
#     # {'sensor_dist':443.8}, #[pix]
#     {'eye_position':0.125}, #[m]
#     {'frame_id': 'camera_link'},
#     {'pixel_step':4},
#     {'pos_x': 0.0},
#     {'pos_y': -0.04},
#     {'pos_z': 0.0},
# ]

# for KUAMS
sensor_params = [
    {'sensor_dist':887.73}, #[pix]
    {'eye_position':0.880}, #[m]
    {'frame_id': 'camera_link'},
    {'pixel_step':4},
    {'pos_x': 0.0},
    {'pos_y': 0.0},
    {'pos_z': 0.0},
]

# for KUAMS
# sensor_params = [
#     {'sensor_dist': 1400.0}, #[pix]
#     {'eye_position': 0.890}, #[m]
#     {'frame_id': 'velodyne'},
#     {'pixel_step': 4},
#     {'pos_x': -0.03},
#     {'pos_y': 0.0},
#     {'pos_z': 0.012},
# ]


def generate_launch_description():

    ld = LaunchDescription()

    # depth_estimation = Node(
    #     package='gbdepth_pkg',
    #     executable = 'depth_estimation_node',
    #     name = 'depth_estimation_node',
    #     parameters = sensor_params,
    #     remappings= [('camera_img', '/camera/camera/color/image_raw')],
    # )
    # ld.add_action(depth_estimation)

    ground_estimation = Node(
        package='gbdepth_pkg',
        executable = 'ground_estimation_node',
        name = 'ground_estimation_node',
    )
    ld.add_action(ground_estimation)

    pcl_estimation = Node(
        package='gbdepth_pkg',
        executable = 'pcl_estimation_node',
        name = 'pcl_estimation_node',
        parameters = sensor_params,
        remappings= [('camera_img', '/camera/camera/color/image_raw')],
    )
    ld.add_action(pcl_estimation)

    return ld