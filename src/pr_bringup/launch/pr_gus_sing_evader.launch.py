import os
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

from numpy import fromstring
from datetime import datetime

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
                        ("ref_pose", "ref_pose_mod"),
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
                    package='pr_ref_gen',
                    node_plugin='pr_ref_gen::RefPose',
                    node_name='ref_pose_gen',
                    remappings=[
                        ("ref_pose", "ref_pose"),
                        ("ref_pose_x", "ref_pose_x"),
                        ("end_flag", "end_flag"),
                        ("joint_position", "joint_position")
                    ],
                    parameters=[
                        {"ref_path": data['general']['ref_path']['x']},
                        {"is_cart": True},
                        {"robot_config_params": data['config_params']['geometry']}
                    ]
                ),

                # ComposableNode(
                #     package='pr_modelling',
                #     node_plugin='pr_modelling::ForwardKinematics',
                #     node_name='for_kin',
                #     remappings=[
                #         ("joint_position", "joint_position"),
                #         ("x_coord", "x_coord"),
                #     ],
                #     parameters=[
                #         {"robot_config_params": pr_config_params},
                #         {"initial_position": first_reference_x},
                #         {"tol": controller_params['dir_kin']['tol']},
                #         {"iter": controller_params['dir_kin']['iter']},
                #     ]
                # ),

                ComposableNode(
                    package='pr_modelling',
                    node_plugin='pr_modelling::ForwardJacobian',
                    node_name='for_jac_med',
                    remappings=[
                        ("x_coord", "x_mocap_sync"),
                        ("for_jac_det", "for_jac_det_med"),
                    ],
                    parameters=[
                        {"robot_config_params": data['config_params']['geometry']},
                    ]
                ),

                ComposableNode(
                    package='pr_modelling',
                    node_plugin='pr_modelling::ForwardJacobian',
                    node_name='for_jac_ref',
                    remappings=[
                        ("x_coord", "ref_pose_x"),
                        ("for_jac_det", "for_jac_det_ref"),
                    ],
                    parameters=[
                        {"robot_config_params": data['config_params']['geometry']},
                    ]
                ),

                ComposableNode(
                    package='pr_modelling',
                    node_plugin='pr_modelling::AngOTS',
                    node_name='ang_ots_med',
                    remappings=[
                        ("x_coord", "x_mocap_sync"),
                        ("ang_ots", "ang_ots_med"),
                    ],
                    parameters=[
                        {"robot_config_params": data['config_params']['geometry']},
                        {"initial_ots": data['sing']['ots']['initial_ots']},
                        {"initial_position": data['general']['init_x']},
                        {"iter_max_ots": data['sing']['ots']['iter']},
                        {"tol_ots": data['sing']['ots']['tol']},
                    ]
                ),

                ComposableNode(
                    package='pr_modelling',
                    node_plugin='pr_modelling::AngOTS',
                    node_name='ang_ots_ref',
                    remappings=[
                        ("x_coord", "ref_pose_x"),
                        ("ang_ots", "ang_ots_ref"),
                    ],
                    parameters=[
                        {"robot_config_params": data['config_params']['geometry']},
                        {"initial_ots": data['sing']['ots']['initial_ots']},
                        {"initial_position": data['general']['init_x']},
                        {"iter_max_ots": data['sing']['ots']['iter']},
                        {"tol_ots": data['sing']['ots']['tol']},
                    ]
                ),

                ComposableNode(
                    package='pr_sing',
                    node_plugin='pr_sing::SingEvader',
                    node_name='sing_evader',
                    remappings=[
                        ("ref_pose", "ref_pose"),
                        ("x_coord", "x_mocap_sync"),
                        ("ang_ots_ref", "ang_ots_ref"),
                        ("ang_ots_med", "ang_ots_med"),
                        ("for_jac_det_ref", "for_jac_det_ref"),
                        ("for_jac_det_med", "for_jac_det_med"),
                        ("ref_pose_mod", "ref_pose_mod")
                    ],
                    parameters=[
                        {"robot_config_params": data['config_params']['geometry']},
                        {"lmin_Ang_OTS": data['sing']['lmin_Ang_OTS']},
                        {"lmin_FJac": data['sing']['lmin_FJac']},
                        {"iter_fk": data['general']['dir_kin']['iter']},
                        {"tol_fk": data['general']['dir_kin']['tol']},
                        {"iter_OTS": data['sing']['ots']['iter']},
                        {"tol_OTS": data['sing']['ots']['tol']},
                        {"ts": data['general']['ts']}
                    ]
                ),

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

                ComposableNode(
                    package='pr_mocap',
                    node_plugin='pr_mocap::PRXMocapSynchronizer',
                    node_name='mocap_synchronizer',
                    remappings=[
                        ("joint_position", "joint_position"),
                        ("x_mocap_sync", "x_mocap_sync")
                    ],
                    parameters=[
                        {"tol": 0.01}
                    ]
                ),

                # ComposableNode(
                #     package='pr_mocap',
                #     node_plugin='pr_mocap::PRXMocapRecorder',
                #     node_name='ref_x_mocap_recorder',
                #     remappings=[
                #         ("end_flag", "end_flag"),
                #         ("joint_position", "joint_position")
                #     ],
                #     parameters=[
                #         {"filename": datetime.now().strftime("%Y_%m_%d-%H_%M_%S") + "_TRR16_CF1_VS2"}
                #     ]
                # ),

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

    return launch.LaunchDescription([pr_gus])