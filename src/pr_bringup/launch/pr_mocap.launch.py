import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

import os

from numpy import fromstring, pi
import yaml

from datetime import datetime

def generate_launch_description():

    """Generate launch description with multiple components."""

    # Load data dictionary
    import sys
    sys.path.append('src/pr_bringup/launch')
    from load_data import data

    pr_mocap = ComposableNodeContainer(
            node_name='pr_mocap',
            node_namespace='',
            package='rclcpp_components',
            node_executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='pr_mocap',
                    node_plugin='pr_mocap::PRXMocap',
                    node_name='mocap',
                    remappings=[
                        ("x_coord_mocap", "x_coord_mocap")
                    ],
                    parameters=[
                        {"server_address": data['mocap_server']["server_address"]},
                        {"server_command_port": data['mocap_server']["server_command_port"]},
                        {"server_data_port": data['mocap_server']["server_data_port"]},
                        {"marker_names":  data['mocap_server']["marker_names"]},
                        {"robot_5p": data['general']['robot']['robot_name']=="robot_5p"},
                    ]
                ),

                # ComposableNode(
                #     package='pr_mocap',
                #     node_plugin='pr_mocap::PRXMocapSynchronizer',
                #     node_name='mocap_synchronizer',
                #     remappings=[
                #         ("joint_position", "joint_position"),
                #         ("x_mocap_sync", "x_mocap_sync")
                #     ],
                #     parameters=[
                #         {"tol": 0.01}
                #     ]
                # ),

                #   ComposableNode(
                #       package='pr_mocap',
                #       node_plugin='pr_mocap::PRXMocapRecorder',
                #       node_name='ref_x_mocap_recorder',
                #       remappings=[
                #           ("end_flag", "end_flag"),
                #           ("joint_position", "joint_position")
                #       ],
                #       parameters=[
                #           {"filename": datetime.now().strftime("%Y_%m_%d-%H_%M_%S") + "_Marina_MVC_2"}
                #       ]
                #   ),           
            ],
            output='screen',
    )

    return launch.LaunchDescription([pr_mocap])