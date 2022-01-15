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
                    node_plugin='pr_sensors_actuators::Motor',
                    node_name='motor_0',
                    remappings=[
                        ("control_action", "control_action"),
                        ("end_flag", "end_flag")
                    ],
                    parameters=[
                        {"vp_conversion": controller_params['vp_conversion'][0]},
                        {"max_v": data['general']['robot']['v_sat']}
                    ]
                ),
                ComposableNode(
                    package='pr_sensors_actuators',
                    node_plugin='pr_sensors_actuators::Motor',
                    node_name='motor_1',
                    remappings=[
                        ("control_action", "control_action"),
                        ("end_flag", "end_flag")
                    ],
                    parameters=[
                        {"vp_conversion": controller_params['vp_conversion'][1]},
                        {"max_v": data['general']['robot']['v_sat']}
                    ]
                ),
                ComposableNode(
                    package='pr_sensors_actuators',
                    node_plugin='pr_sensors_actuators::Motor',
                    node_name='motor_2',
                    remappings=[
                        ("control_action", "control_action"),
                        ("end_flag", "end_flag")
                    ],
                    parameters=[
                        {"vp_conversion": controller_params['vp_conversion'][2]},
                        {"max_v": data['general']['robot']['v_sat']}
                    ]
                ),
                ComposableNode(
                    package='pr_sensors_actuators',
                    node_plugin='pr_sensors_actuators::Motor',
                    node_name='motor_3',
                    remappings=[
                        ("control_action", "control_action"),
                        ("end_flag", "end_flag")
                    ],
                    parameters=[
                        {"vp_conversion": controller_params['vp_conversion'][3]},
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
                        {"k2": controller_params['controller']['k2']},
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
                    package='pr_aux',
                    node_plugin='pr_aux::KalmanFilter',
                    node_name='kalman_filter',
                    remappings=[
                        ("joint_position", "joint_position"),
                        ("joint_position_filt", "joint_position_filt"),
                        ("joint_velocity_filt", "joint_velocity_filt")
                    ],
                    parameters=[
                        {"initial_value": data['general']['init_q']},
                        {"ts": data['general']['ts']},
                        {"q": 100.0},
                        {"r": 0.001}
                    ]
                ),

                ComposableNode(
                    package='pr_lwpr',
                    node_plugin='pr_lwpr::LWPRFwd',
                    node_name='lwpr_fwd',
                    remappings=[
                        ("joint_position", "joint_position_filt"),
                        ("joint_velocity", "joint_velocity_filt"),
                        ("control_action", "control_action"),
                        ("out_lwpr_fwd", "out_lwpr_fwd")
                    ],
                    parameters=[
                        {"initD": data['lwpr']['fwd']['initD']},
                        {"initAlpha": data['lwpr']['fwd']['initAlpha']},
                        {"penalty": data['lwpr']['fwd']['penalty']},
                        {"initLambda": data['lwpr']['fwd']['initLambda']},
                        {"finalLambda": data['lwpr']['fwd']['finalLambda']},
                        {"activateLearning": data['lwpr']['fwd']['activateLearning']},
                        {"loadModel": data['lwpr']['fwd']['loadModel']},
                        {"saveModel": data['lwpr']['fwd']['saveModel']}
                    ]
                ),
                
                ComposableNode(
                    package='pr_lwpr',
                    node_plugin='pr_lwpr::LWPRInv',
                    node_name='lwpr_inv',
                    remappings=[
                        ("ref_pose", "ref_pose"),
                        ("control_action", "control_action"),
                        ("out_lwpr_inv", "out_lwpr_inv")
                    ],
                    parameters=[
                        {"initD": data['lwpr']['inv']['initD']},
                        {"initAlpha": data['lwpr']['inv']['initAlpha']},
                        {"penalty": data['lwpr']['inv']['penalty']},
                        {"initLambda": data['lwpr']['inv']['initLambda']},
                        {"finalLambda": data['lwpr']['inv']['finalLambda']},
                        {"activateLearning": data['lwpr']['inv']['activateLearning']},
                        {"loadModel": data['lwpr']['inv']['loadModel']},
                        {"saveModel": data['lwpr']['inv']['saveModel']},
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
                    node_plugin='pr_sensors_actuators::Encoders',
                    node_name='position_sensors',
                    remappings=[
                        ("joint_position", "joint_position")
                    ],
                    parameters=[
                        {"ts_ms": data['general']['ts']*1000},
                        {"initial_position": data['general']['init_q']},
                        {"gearbox_mult":  data['general']['robot']['encoder_gearbox']},
                    ]
                ),
            ],
            output='screen',
    )

    # ros2_bag = launch.actions.ExecuteProcess(
    #     cmd=['ros2', 'bag', 'record', '-a'],
    #     output = 'screen'
    # )

    return launch.LaunchDescription([pr_gus])