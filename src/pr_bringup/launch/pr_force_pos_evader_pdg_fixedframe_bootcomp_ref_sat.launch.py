import os
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

from numpy import fromstring

import yaml


def generate_launch_description():

    # Load data dictionary
    import sys
    sys.path.append('src/pr_bringup/launch')
    from load_data import data

    controller_params = data['pdg_pid']

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
                        ("pos_admittance", "pos_admittance"),
                        ("activation_pin", "sing_pin")
                    ],
                    parameters=[
                        {"mass": data['force']['admittance_params']['mass']},
                        {"damping": data['force']['admittance_params']['damping']},
                        {"stiffness": data['force']['admittance_params']['stiffness']},
                        {"ts": data['general']['ts']}
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
                    package='pr_modelling',
                    node_plugin='pr_modelling::InverseKinematics',
                    node_name='inv_kin',
                    remappings=[
                        ("x_coord", "x_mocap_sync"),
                        ("q_sol", "q_sol"),
                    ],
                    parameters=[
                        {"robot_config_params": data['config_params']['geometry']},
                    ]
                ),

                ComposableNode(
                    package='pr_modelling',
                    node_plugin='pr_modelling::IndependentJacobian',
                    node_name='ind_jac',
                    remappings=[
                        ("q_sol", "q_sol"),
                        ("ind_jac", "ind_jac"),
                    ],
                    parameters=[
                    ]
                ),

                ComposableNode(
                    package='pr_modelling',
                    node_plugin='pr_modelling::DependentJacobian',
                    node_name='dep_jac',
                    remappings=[
                        ("x_coord", "x_mocap_sync"),
                        ("q_sol", "q_sol"),
                        ("dep_jac", "dep_jac")
                    ],
                    parameters=[
                        {"robot_config_params": data['config_params']['geometry']},
                    ]
                ),

                ComposableNode(
                    package='pr_modelling',
                    node_plugin='pr_modelling::RastT',
                    node_name='rast_t',
                    remappings=[
                        ("dep_jac", "dep_jac"),
                        ("ind_jac", "ind_jac"),
                        ("rast_t", "rast_t")
                    ],
                    parameters=[
                    ]
                ),

                ComposableNode(
                    package='pr_modelling',
                    node_plugin='pr_modelling::QGrav',
                    node_name='q_grav',
                    remappings=[
                        ("x_coord", "x_mocap_sync"),
                        ("q_sol", "q_sol"),
                        ("rast_t", "rast_t")
                    ],
                    parameters=[
                        {"p11": data['config_params']['physical_properties']['p11']},
                        {"p12": data['config_params']['physical_properties']['p12']},
                        {"p21": data['config_params']['physical_properties']['p21']},
                        {"p22": data['config_params']['physical_properties']['p22']},
                        {"p31": data['config_params']['physical_properties']['p31']},
                        {"p32": data['config_params']['physical_properties']['p32']},
                        {"p41": data['config_params']['physical_properties']['p41']},
                        {"p42": data['config_params']['physical_properties']['p42']},
                        {"pm":  data['config_params']['physical_properties']['pm']},
                    ]
                ),

                ComposableNode(
                    package='pr_controllers',
                    node_plugin='pr_controllers::PDGController',
                    node_name='controller',
                    remappings=[
                        ("ref_pose", "ref_pose_mod"),
                        ("joint_position", "joint_position"),
                        ("joint_velocity", "joint_velocity"),
                        ("q_grav", "q_grav")
                    ],
                    parameters=[
                        {"kp_gain": controller_params['controller']['kp']},
                        {"kv_gain": controller_params['controller']['kv']},
                        {"initial_position": data['general']['init_q']},
                        {"initial_reference": data['general']['init_q']}
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

                ComposableNode(
                    package='pr_ref_gen',
                    node_plugin='pr_ref_gen::RefPose',
                    node_name='ref_force_gen',
                    remappings=[
                        ("ref_pose", "ref_force"),
                        ("end_flag", "end_flag_force"),
                        ("joint_position", "joint_position"),
                        ("ref_pose_x", "useless_topic")
                    ],
                    parameters=[
                        {"ref_path": data['force']['ref_force_path']},
                        {"is_cart": False},
                        {"robot_config_params": data['config_params']['geometry']}
                    ]
                ),


                ComposableNode(
                    package='pr_aux',
                    node_plugin='pr_aux::AddGain',
                    node_name='add_gain',
                    remappings=[
                        ("input1", "pos_admittance"),
                        ("input2", "ref_pose_x"),
                        ("output", "ref_sum_x")
                    ],
                    parameters=[
                        {"signs": [1.0, 1.0]},
                        {"gains": [1.0, 1.0, 1.0, 1.0]}
                    ]
                ),

                ComposableNode(
                    package='pr_modelling',
                    node_plugin='pr_modelling::InverseKinematics',
                    node_name='inv_kin_ref',
                    remappings=[
                        ("x_coord", "ref_sum_x"),
                        ("q_inde_sol", "ref_sum"),
                    ],
                    parameters=[
                        {"robot_config_params": data['config_params']['geometry']},
                    ]
                ),

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
                        ("x_coord", "ref_sum_x"),
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
                        ("x_coord", "ref_sum_x"),
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
                        ("ref_pose", "ref_sum"),
                        ("x_coord", "x_mocap_sync"),
                        ("ang_ots_ref", "ang_ots_ref"),
                        ("ang_ots_med", "ang_ots_med"),
                        ("for_jac_det_ref", "for_jac_det_ref"),
                        ("for_jac_det_med", "for_jac_det_med"),
                        ("ref_pose_mod", "ref_pose_mod_not_sat"),
                        ("sing_pin", "sing_pin")
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
                    package='pr_aux',
                    node_plugin='pr_aux::Saturator',
                    node_name='saturator',
                    remappings=[
                        ("signal_init", "ref_pose_mod_not_sat"),
                        ("signal_saturated", "ref_pose_mod")
                    ],
                    parameters=[
                        {"min_val": data['config_params']['q_lim']['min']},
                        {"max_val": data['config_params']['q_lim']['max']}
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

    return launch.LaunchDescription([pr_gus])