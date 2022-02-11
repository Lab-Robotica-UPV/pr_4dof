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

    controller_params = data['pdg_pid']
    
    pr_dmp = ComposableNodeContainer(
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
                        {"max_v":  data['general']['robot']['v_sat']}
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
                        {"max_v":  data['general']['robot']['v_sat']}
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
                        {"max_v":  data['general']['robot']['v_sat']}
                    ]
                ),
                ComposableNode(
                    package='pr_dmp',
                    node_plugin='pr_dmp::DmpRefForce',
                    node_name='dmp_ref_force',
                    remappings=[
                        ("joint_position", "joint_position"),
                        ("dmp_ref_gen_q", "ref_pose_not_sat"),
                        ("dmp_ref_gen_x", "ref_pose_x"),
                        ("dmp_force_pos", "dmp_force"),
                        ("dmp_force_force", "dmp_force_force"),
                        ("force_traj","ref_force")
                    ],
                    parameters=[
                        {"ts": data['general']['ts']},
                        {"ref_path": data['general']['ref_path']['q']},
                        {"force_path": data['force']['ref_force_path']},
                        {"n_basis_functions": 200},
                        {"save_directory": ""},
                        {"isCart": False},
                        {"calcCart": True},
                        {"ref_x_init": data['general']['init_x']},
                        {"robot_config_params": data['config_params']['geometry']},
                        {"damping_coefficient_pos": data['dmp']['damping']},
                        {"spring_constant_pos": data['dmp']['spring']},
                        {"mass_pos": data['dmp']['mass']},
                        {"damping_coefficient_force": [1.0, 0.1, 1.0, 1.0]},
                        {"spring_constant_force": [1.0, 0.1, 1.0, 1.0]},
                        {"mass_force": [0.25, 0.025, 0.25, 0.25]},
                        {"speed": data['dmp']['speed']},
                        {"gain_slowdown": data['dmp']['gain_slowdown']}
                    ]
                ),

                ComposableNode(
                    package='pr_dmp',
                    node_plugin='pr_dmp::ForceAdapted',
                    node_name='dmp_force_adapted',
                    remappings=[
                        ("ref_force", "ref_force"),
                        ("force_state", "force_state_fixed"),
                        ("force_adapted", "dmp_force_force")
                    ],
                ),

                ComposableNode(
                    package='pr_aux',
                    node_plugin='pr_aux::Saturator',
                    node_name='saturator',
                    remappings=[
                        ("signal_init", "ref_pose_not_sat"),
                        ("signal_saturated", "ref_pose")
                    ],
                    parameters=[
                        {"min_val": data['config_params']['q_lim']['min']},
                        {"max_val": data['config_params']['q_lim']['max']}
                    ]
                ),

                ComposableNode(
                    package='pr_modelling',
                    node_plugin='pr_modelling::ForwardJacobian',
                    node_name='for_jac_med',
                    remappings=[
                        ("x_coord", "x_mocap_sync"),
                        ("for_jac_det", "for_jac_det_med"),
                        ("for_jac", "for_jac")
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
                    package='pr_dmp',
                    node_plugin='pr_dmp::EvaderForce',
                    node_name='dmp_evader_force',
                    remappings=[
                        ("ref_pose", "ref_pose"),
                        ("x_coord", "x_mocap_sync"),
                        ("ang_ots_med", "ang_ots_med"),
                        ("for_jac_det_med", "for_jac_det_med"),
                        ("evader_force", "evader_force"),
                        ("sing_pin", "sing_pin")
                    ],
                    parameters=[
                        {"robot_config_params": data['config_params']['geometry']},
                        {"iter_fk": data['general']['dir_kin']['iter']},
                        {"tol_fk": data['general']['dir_kin']['tol']},
                        {"iter_OTS": data['sing']['ots']['iter']},
                        {"tol_OTS": data['sing']['ots']['tol']},
                        {"lmin_Ang_OTS": data['sing']['lmin_Ang_OTS']},
                        {"lmin_FJac": data['sing']['lmin_FJac']},
                        {"ts": data['general']['ts']},
                        {"max_force": 150.0},
                        {"inc_force_jac": 1.0},
                        {"inc_force_OTS": 1.0},
                        {"beta": 5.0},
                        {"decay": 1.0/400.0},
                    ]
                ),

                ComposableNode(
                    package='pr_sensors_actuators',
                    node_plugin='pr_sensors_actuators::ForceSensor',
                    node_name='force_sensor',
                    remappings=[
                        ("joint_position", "joint_position"),
                        ("force_state", "force_state"),
                        ("force_state_sync", "force_state_sync"),
                        ("force_state_accelstamped", "force_state_accelstamped")
                    ],
                    parameters=[
                        {"calibration": data['force']['calibration']},
                        {"noise_threshold": data['force']['noise_threshold']}
                    ]
                ),
                ComposableNode(
                    package='pr_modelling',
                    node_plugin='pr_modelling::ForceFixedFrame',
                    node_name='force_fixed_frame',
                    remappings=[
                        ("force_state", "force_state_sync"),
                        ("x_coord", "x_mocap_sync"),
                        ("force_state_fixed", "force_state_fixed"),
                    ],
                    parameters=[
                        {"boot_mass": data['force']['boot_mass']},
                        {"boot_cdg": data['force']['boot_cdg']},
                        {"boot_compensation": data['force']['boot_compensation']},
                        {"fixed_frame_noise_threshold": data['force']['fixed_frame_noise_threshold']}
                    ]
                ),
                ComposableNode(
                    package='pr_modelling',
                    node_plugin='pr_modelling::Admittance',
                    node_name='admittance',
                    remappings=[
                        ("force_state", "force_state_fixed"),
                        ("ref_force", "ref_force"),
                        ("vel_admittance", "vel_admittance"),
                        ("pos_admittance", "force_hs"),
                        ("activation_pin", "sing_pin")
                    ],
                    parameters=[
                        {"mass": data['dmp']['params_H']['mass']},
                        {"damping": data['dmp']['params_H']['damping']},
                        {"stiffness": data['dmp']['params_H']['spring']},
                        {"ts": data['general']['ts']}
                    ]
                ),

                ComposableNode(
                    package='pr_aux',
                    node_plugin='pr_aux::MatrixMult',
                    node_name='force_cart_to_joints',
                    remappings=[
                        ("input_vector", "force_hs"),
                        ("input_matrix", "for_jac"),
                        ("output_vector", "admittance_force")
                    ],
                    parameters=[
                    ]
                ),

                ComposableNode(
                    package='pr_aux',
                    node_plugin='pr_aux::AddGain',
                    node_name='add_gain',
                    remappings=[
                        ("input1", "evader_force"),
                        ("input2", "admittance_force"),
                        ("output", "dmp_force")
                    ],
                    parameters=[
                        {"signs": [1.0, 1.0]},
                        {"gains": [1.0, 1.0, 1.0, 1.0]}
                    ]
                ),
                
                ComposableNode(
                    package='pr_controllers',
                    node_plugin='pr_controllers::PIDController',
                    node_name='controller',
                    remappings=[
                        ("ref_pos", "ref_pose"),
                        ("pos", "joint_position"),
                    ],
                    parameters=[
                        {"kp_gain": controller_params['controller']['kp']},
                        {"kv_gain": controller_params['controller']['kv']},
                        {"ki_gain": controller_params['controller']['ki']}
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

    return launch.LaunchDescription([pr_dmp])