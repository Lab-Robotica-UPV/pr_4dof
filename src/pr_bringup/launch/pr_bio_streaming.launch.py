import os
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

from numpy import fromstring

import yaml

def generate_launch_description():

    """Generate launch description with multiple components."""

    #Load config file

    controller_params_file = os.path.join(
        get_package_share_directory('pr_bringup'),
        'config',
        'pr_gus.yaml'
    )

    force_file = os.path.join(
        get_package_share_directory('pr_bringup'),
        'config',
        'pr_force.yaml'
    )

    controller_yaml_file = open(controller_params_file)
    controller_params = yaml.load(controller_yaml_file)

    force_yaml_file = open(force_file)
    force_params = yaml.load(force_yaml_file)

    
    pr_bio_streaming = ComposableNodeContainer(
            node_name='pr_container',
            node_namespace='',
            package='rclcpp_components',
            node_executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='pr_biomech',
                    node_plugin='pr_biomech::StreamingGDLF',
                    node_name='streaming_GDLF',
                    remappings=[
                        ("force_state_sync", "force_state_sync"),
                    ],
                ),
                ComposableNode(
                    package='pr_sensors_actuators',
                    node_plugin='pr_sensors_actuators::ForceSensor',
                    node_name='force_sensor',
                    remappings=[
                        ("force_state", "force_state"),
                        ("force_state_sync", "force_state_sync"),
                        ("force_state_accelstamped", "force_state_accelstamped")
                    ],
                    parameters=[
                        {"calibration": force_params['calibration']}
                    ]
                ),
                ComposableNode(
                    package='pr_sensors_actuators',
                    node_plugin='pr_sensors_actuators::Encoders',
                    node_name='position_sensors',
                    remappings=[
                        ("joint_position", "joint_position")
                    ],
                    parameters=[
                        {"ts_ms": controller_params['ts']*1000},
                        # {"initial_position": [0, 0, 0, 0]}
                    ]
                ),
            ],
            output='screen',
    )

    return launch.LaunchDescription([pr_bio_streaming])