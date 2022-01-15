import os
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

from numpy import fromstring

import yaml


def generate_launch_description():

    """Generate launch description with multiple components."""

    # Load data dictionary
    import sys
    sys.path.append('src/pr_bringup/launch')
    from load_data import data

    controller_params = data['gus']
    
    pr_gus = ComposableNodeContainer(
            node_name='pr_container',
            node_namespace='',
            package='rclcpp_components',
            node_executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='pr_sensors_actuators',
                    node_plugin='pr_sensors_actuators::MotorsSimulink',
                    node_name='motors',
                    remappings=[
                        ("control_action", "control_action"),
                        ("voltaje_sim", "voltaje_sim")
                    ],
                    parameters=[
                        {"vp_conversion": controller_params['vp_conversion'][0]},
                        {"max_v": data['general']['robot']['v_sat']}
                    ]
                ),
                ComposableNode(
                    package='pr_controllers',
                    node_plugin='pr_controllers::GusController',
                    node_name='controller',
                    remappings=[
                        ("ref_pose", "ref_pose"),
                        ("joint_position", "joint_position"),
                        ("joint_velocity", "joint_velocity")
                    ],
                    parameters=[
                        {"k1": controller_params['controller']['k1']},
                        {"k2": controller_params['controller']['k1']},
                        {"ts": data['general']['ts']},
                        {"initial_position": data['general']['init_q']},
                        {"initial_reference": data['general']['init_q']}
                    ]
                ),
                ComposableNode(
                    package='pr_aux',
                    node_plugin='pr_aux::Derivator',
                    node_name='derivator',
                    remappings=[
                        ("joint_position", "joint_position"),
                        ("joint_velocity", "joint_velocity")
                    ],
                    parameters=[
                        {"initial_value": data['general']['init_q']},
                        {"ts": data['general']['ts']}
                    ]
                ),
                ComposableNode(
                    package='pr_ref_gen',
                    node_plugin='pr_ref_gen::RefPose',
                    node_name='ref_pose_gen',
                    remappings=[
                        ("ref_pose", "ref_pose"),
                        ("end_flag", "end_flag"),
                        ("joint_position", "joint_position")
                    ],
                    parameters=[
                        {"ref_path": data['general']['ref_path']['q']},
                        {"is_cart": False},
                        {"robot_config_params": data['config_params']['geometry']}
                    ]
                ),
                ComposableNode(
                    package='pr_sensors_actuators',
                    node_plugin='pr_sensors_actuators::EncodersSimulink',
                    node_name='position_sensors',
                    remappings=[
                        ("joint_position", "joint_position")
                    ],
                    parameters=[
                        {"ts_ms": data['general']['ts']*1000},
                        {"initial_position": data['general']['init_q']}
                    ]
                ),
            ],
            output='screen',
    )

    return launch.LaunchDescription([pr_gus])