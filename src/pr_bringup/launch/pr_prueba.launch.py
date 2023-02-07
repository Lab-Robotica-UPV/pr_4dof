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

    controller_params = data['pdg_pid']
    print(data['general']['init_q'])

    intra_bool = False

    pr_pid = ComposableNodeContainer(
            node_name='pr_container',
            node_namespace='',
            package='rclcpp_components',
            node_executable='component_container',
            composable_node_descriptions=[
                # ComposableNode(
                #     package='pr_sensors_actuators',
                #     node_plugin='pr_sensors_actuators::Motor',
                #     node_name='motor_0',
                #     remappings=[
                #         ("control_action", "control_action"),
                #         ("end_flag", "end_flag")
                #     ],
                #     parameters=[
                #         {"vp_conversion": controller_params['vp_conversion'][0]},
                #         {"max_v": data['general']['robot']['v_sat']}
                #     ]
                # ),
                # ComposableNode(
                #     package='pr_sensors_actuators',
                #     node_plugin='pr_sensors_actuators::Motor',
                #     node_name='motor_1',
                #     remappings=[
                #         ("control_action", "control_action"),
                #         ("end_flag", "end_flag")
                #     ],
                #     parameters=[
                #         {"vp_conversion": controller_params['vp_conversion'][1]},
                #         {"max_v": data['general']['robot']['v_sat']}
                #     ]
                # ),
                # ComposableNode(
                #     package='pr_sensors_actuators',
                #     node_plugin='pr_sensors_actuators::Motor',
                #     node_name='motor_2',
                #     remappings=[
                #         ("control_action", "control_action"),
                #         ("end_flag", "end_flag")
                #     ],
                #     parameters=[
                #         {"vp_conversion": controller_params['vp_conversion'][2]},
                #         {"max_v": data['general']['robot']['v_sat']}
                #     ]
                # ),
                # ComposableNode(
                #     package='pr_sensors_actuators',
                #     node_plugin='pr_sensors_actuators::Motor',
                #     node_name='motor_3',
                #     remappings=[
                #         ("control_action", "control_action"),
                #         ("end_flag", "end_flag")
                #     ],
                #     parameters=[
                #         {"vp_conversion": controller_params['vp_conversion'][3]},
                #         {"max_v": data['general']['robot']['v_sat']}
                #     ]
                # ),
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
                    ],
                    extra_arguments=[{'use_intra_process_comms': intra_bool}]
                ),
                ComposableNode(
                    package='pr_controllers',
                    node_plugin='pr_controllers::PIDController',
                    node_name='controller',
                    remappings=[
                        ("ref_pos", "ref_pose"),
                        ("pos", "joint_position"),
                        ("control_action", "control_action"),
                    ],
                    parameters=[
                        {"kp_gain": controller_params['controller']['kp']},
                        {"kv_gain": controller_params['controller']['kv']},
                        {"ki_gain": controller_params['controller']['ki']}
                    ],
                    extra_arguments=[{'use_intra_process_comms': intra_bool}]
                ),
                ComposableNode(
                    package='pr_controllers',
                    node_plugin='pr_controllers::PIDController',
                    node_name='controller2',
                    remappings=[
                        ("ref_pos", "ref_pose"),
                        ("pos", "joint_position"),
                        ("control_action", "control_action2"),
                    ],
                    parameters=[
                        {"kp_gain": controller_params['controller']['kp']},
                        {"kv_gain": controller_params['controller']['kv']},
                        {"ki_gain": controller_params['controller']['ki']}
                    ],
                    extra_arguments=[{'use_intra_process_comms': intra_bool}]
                ),
                ComposableNode(
                    package='pr_controllers',
                    node_plugin='pr_controllers::PIDController',
                    node_name='controller3',
                    remappings=[
                        ("ref_pos", "ref_pose"),
                        ("pos", "joint_position"),
                        ("control_action", "control_action3"),
                    ],
                    parameters=[
                        {"kp_gain": controller_params['controller']['kp']},
                        {"kv_gain": controller_params['controller']['kv']},
                        {"ki_gain": controller_params['controller']['ki']}
                    ],
                    extra_arguments=[{'use_intra_process_comms': intra_bool}]
                ),
                ComposableNode(
                    package='pr_controllers',
                    node_plugin='pr_controllers::PIDController',
                    node_name='controller4',
                    remappings=[
                        ("ref_pos", "ref_pose"),
                        ("pos", "joint_position"),
                        ("control_action", "control_action4"),
                    ],
                    parameters=[
                        {"kp_gain": controller_params['controller']['kp']},
                        {"kv_gain": controller_params['controller']['kv']},
                        {"ki_gain": controller_params['controller']['ki']}
                    ],
                    extra_arguments=[{'use_intra_process_comms': intra_bool}]
                ),
                ComposableNode(
                    package='pr_controllers',
                    node_plugin='pr_controllers::PIDController',
                    node_name='controller5',
                    remappings=[
                        ("ref_pos", "ref_pose"),
                        ("pos", "joint_position"),
                        ("control_action", "control_action5"),
                    ],
                    parameters=[
                        {"kp_gain": controller_params['controller']['kp']},
                        {"kv_gain": controller_params['controller']['kv']},
                        {"ki_gain": controller_params['controller']['ki']}
                    ],
                    extra_arguments=[{'use_intra_process_comms': intra_bool}]
                ),
                ComposableNode(
                    package='pr_controllers',
                    node_plugin='pr_controllers::PIDController',
                    node_name='controller6',
                    remappings=[
                        ("ref_pos", "ref_pose"),
                        ("pos", "joint_position"),
                        ("control_action", "control_action6"),
                    ],
                    parameters=[
                        {"kp_gain": controller_params['controller']['kp']},
                        {"kv_gain": controller_params['controller']['kv']},
                        {"ki_gain": controller_params['controller']['ki']}
                    ],
                    extra_arguments=[{'use_intra_process_comms': intra_bool}]
                ),
                ComposableNode(
                    package='pr_controllers',
                    node_plugin='pr_controllers::PIDController',
                    node_name='controller7',
                    remappings=[
                        ("ref_pos", "ref_pose"),
                        ("pos", "joint_position"),
                        ("control_action", "control_action7"),
                    ],
                    parameters=[
                        {"kp_gain": controller_params['controller']['kp']},
                        {"kv_gain": controller_params['controller']['kv']},
                        {"ki_gain": controller_params['controller']['ki']}
                    ],
                    extra_arguments=[{'use_intra_process_comms': intra_bool}]
                ),
                ComposableNode(
                    package='pr_controllers',
                    node_plugin='pr_controllers::PIDController',
                    node_name='controller8',
                    remappings=[
                        ("ref_pos", "ref_pose"),
                        ("pos", "joint_position"),
                        ("control_action", "control_action8"),
                    ],
                    parameters=[
                        {"kp_gain": controller_params['controller']['kp']},
                        {"kv_gain": controller_params['controller']['kv']},
                        {"ki_gain": controller_params['controller']['ki']}
                    ],
                    extra_arguments=[{'use_intra_process_comms': intra_bool}]
                ),
                ComposableNode(
                    package='pr_controllers',
                    node_plugin='pr_controllers::PIDController',
                    node_name='controller9',
                    remappings=[
                        ("ref_pos", "ref_pose"),
                        ("pos", "joint_position"),
                        ("control_action", "control_action9"),
                    ],
                    parameters=[
                        {"kp_gain": controller_params['controller']['kp']},
                        {"kv_gain": controller_params['controller']['kv']},
                        {"ki_gain": controller_params['controller']['ki']}
                    ],
                    extra_arguments=[{'use_intra_process_comms': intra_bool}]
                ),
                ComposableNode(
                    package='pr_controllers',
                    node_plugin='pr_controllers::PIDController',
                    node_name='controller10',
                    remappings=[
                        ("ref_pos", "ref_pose"),
                        ("pos", "joint_position"),
                        ("control_action", "control_action10"),
                    ],
                    parameters=[
                        {"kp_gain": controller_params['controller']['kp']},
                        {"kv_gain": controller_params['controller']['kv']},
                        {"ki_gain": controller_params['controller']['ki']}
                    ],
                    extra_arguments=[{'use_intra_process_comms': intra_bool}]
                ),
                ComposableNode(
                    package='pr_controllers',
                    node_plugin='pr_controllers::PIDController',
                    node_name='controller11',
                    remappings=[
                        ("ref_pos", "ref_pose"),
                        ("pos", "joint_position"),
                        ("control_action", "control_action11"),
                    ],
                    parameters=[
                        {"kp_gain": controller_params['controller']['kp']},
                        {"kv_gain": controller_params['controller']['kv']},
                        {"ki_gain": controller_params['controller']['ki']}
                    ],
                    extra_arguments=[{'use_intra_process_comms': intra_bool}]
                ),
                ComposableNode(
                    package='pr_controllers',
                    node_plugin='pr_controllers::PIDController',
                    node_name='controller12',
                    remappings=[
                        ("ref_pos", "ref_pose"),
                        ("pos", "joint_position"),
                        ("control_action", "control_action12"),
                    ],
                    parameters=[
                        {"kp_gain": controller_params['controller']['kp']},
                        {"kv_gain": controller_params['controller']['kv']},
                        {"ki_gain": controller_params['controller']['ki']}
                    ],
                    extra_arguments=[{'use_intra_process_comms': intra_bool}]
                ),
                ComposableNode(
                    package='pr_controllers',
                    node_plugin='pr_controllers::PIDController',
                    node_name='controller13',
                    remappings=[
                        ("ref_pos", "ref_pose"),
                        ("pos", "joint_position"),
                        ("control_action", "control_action13"),
                    ],
                    parameters=[
                        {"kp_gain": controller_params['controller']['kp']},
                        {"kv_gain": controller_params['controller']['kv']},
                        {"ki_gain": controller_params['controller']['ki']}
                    ],
                    extra_arguments=[{'use_intra_process_comms': intra_bool}]
                ),
                ComposableNode(
                    package='pr_controllers',
                    node_plugin='pr_controllers::PIDController',
                    node_name='controller14',
                    remappings=[
                        ("ref_pos", "ref_pose"),
                        ("pos", "joint_position"),
                        ("control_action", "control_action14"),
                    ],
                    parameters=[
                        {"kp_gain": controller_params['controller']['kp']},
                        {"kv_gain": controller_params['controller']['kv']},
                        {"ki_gain": controller_params['controller']['ki']}
                    ],
                    extra_arguments=[{'use_intra_process_comms': intra_bool}]
                ),
                ComposableNode(
                    package='pr_controllers',
                    node_plugin='pr_controllers::PIDController',
                    node_name='controller15',
                    remappings=[
                        ("ref_pos", "ref_pose"),
                        ("pos", "joint_position"),
                        ("control_action", "control_action15"),
                    ],
                    parameters=[
                        {"kp_gain": controller_params['controller']['kp']},
                        {"kv_gain": controller_params['controller']['kv']},
                        {"ki_gain": controller_params['controller']['ki']}
                    ],
                    extra_arguments=[{'use_intra_process_comms': intra_bool}]
                ),
                ComposableNode(
                    package='pr_controllers',
                    node_plugin='pr_controllers::PIDController',
                    node_name='controller16',
                    remappings=[
                        ("ref_pos", "ref_pose"),
                        ("pos", "joint_position"),
                        ("control_action", "control_action16"),
                    ],
                    parameters=[
                        {"kp_gain": controller_params['controller']['kp']},
                        {"kv_gain": controller_params['controller']['kv']},
                        {"ki_gain": controller_params['controller']['ki']}
                    ],
                    extra_arguments=[{'use_intra_process_comms': intra_bool}]
                ),
                ComposableNode(
                    package='pr_controllers',
                    node_plugin='pr_controllers::PIDController',
                    node_name='controller17',
                    remappings=[
                        ("ref_pos", "ref_pose"),
                        ("pos", "joint_position"),
                        ("control_action", "control_action17"),
                    ],
                    parameters=[
                        {"kp_gain": controller_params['controller']['kp']},
                        {"kv_gain": controller_params['controller']['kv']},
                        {"ki_gain": controller_params['controller']['ki']}
                    ],
                    extra_arguments=[{'use_intra_process_comms': intra_bool}]
                ),
                ComposableNode(
                    package='pr_controllers',
                    node_plugin='pr_controllers::PIDController',
                    node_name='controller18',
                    remappings=[
                        ("ref_pos", "ref_pose"),
                        ("pos", "joint_position"),
                        ("control_action", "control_action18"),
                    ],
                    parameters=[
                        {"kp_gain": controller_params['controller']['kp']},
                        {"kv_gain": controller_params['controller']['kv']},
                        {"ki_gain": controller_params['controller']['ki']}
                    ],
                    extra_arguments=[{'use_intra_process_comms': intra_bool}]
                ),
                ComposableNode(
                    package='pr_controllers',
                    node_plugin='pr_controllers::PIDController',
                    node_name='controller19',
                    remappings=[
                        ("ref_pos", "ref_pose"),
                        ("pos", "joint_position"),
                        ("control_action", "control_action19"),
                    ],
                    parameters=[
                        {"kp_gain": controller_params['controller']['kp']},
                        {"kv_gain": controller_params['controller']['kv']},
                        {"ki_gain": controller_params['controller']['ki']}
                    ],
                    extra_arguments=[{'use_intra_process_comms': intra_bool}]
                ),
                ComposableNode(
                    package='pr_controllers',
                    node_plugin='pr_controllers::PIDController',
                    node_name='controller20',
                    remappings=[
                        ("ref_pos", "ref_pose"),
                        ("pos", "joint_position"),
                        ("control_action", "control_action20"),
                    ],
                    parameters=[
                        {"kp_gain": controller_params['controller']['kp']},
                        {"kv_gain": controller_params['controller']['kv']},
                        {"ki_gain": controller_params['controller']['ki']}
                    ],
                    extra_arguments=[{'use_intra_process_comms': intra_bool}]
                ),
                ComposableNode(
                    package='pr_controllers',
                    node_plugin='pr_controllers::GusController',
                    node_name='controller21',
                    remappings=[
                        ("ref_pose", "ref_pose"),
                        ("joint_position", "joint_position"),
                        ("control_action", "control_action21")
                        # ("joint_velocity", "joint_velocity")
                    ],
                    parameters=[
                        {"k1": data['gus']['controller']['k1']},
                        {"k2": data['gus']['controller']['k2']},
                        {"ts": data['general']['ts']},
                        {"initial_position": data['general']['init_q']},
                        {"initial_reference": data['general']['init_q']}
                    ],
                    extra_arguments=[{'use_intra_process_comms': intra_bool}]
                ),
                # ComposableNode(
                #     package='pr_mocap',
                #     node_plugin='pr_mocap::PRXMocap',
                #     node_name='mocap',
                #     remappings=[
                #         ("x_coord_mocap", "x_coord_mocap")
                #     ],
                #     parameters=[
                #         {"server_address": data['mocap_server']["server_address"]},
                #         {"server_command_port": data['mocap_server']["server_command_port"]},
                #         {"server_data_port": data['mocap_server']["server_data_port"]},
                #         {"marker_names":  data['mocap_server']["marker_names"]},
                #         {"robot_5p": data['general']['robot']['robot_name']=="robot_5p"},
                #     ]
                # ),

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

                # ComposableNode(
                #     package='pr_mocap',
                #     node_plugin='pr_mocap::PRXMocapRecorder',
                #     node_name='ref_x_mocap_recorder',
                #     remappings=[
                #         ("end_flag", "end_flag"),
                #         ("joint_position", "joint_position")
                #     ],
                #     parameters=[
                #         {"filename": datetime.now().strftime("%Y_%m_%d-%H_%M_%S") + "_PauMid"}
                #     ]
                # ),

                # ComposableNode(
                #     package='pr_sensors_actuators',
                #     node_plugin='pr_sensors_actuators::ForceSensor',
                #     node_name='force_sensor',
                #     remappings=[
                #         ("joint_position", "joint_position"),
                #         ("force_state", "force_state"),
                #         ("force_state_sync", "force_state_sync"),
                #         ("force_state_accelstamped", "force_state_accelstamped")
                #     ],
                #     parameters=[
                #         {"calibration": data['force']['calibration']},
                #         {"noise_threshold": data['force']['noise_threshold']}
                #     ]
                # ),

                # ComposableNode(
                #     package='pr_modelling',
                #     node_plugin='pr_modelling::ForceFixedFrame',
                #     node_name='force_fixed_frame',
                #     remappings=[
                #         ("force_state", "force_state_sync"),
                #         ("x_coord", "x_coord"),
                #         ("force_state_fixed", "force_state_fixed"),
                #         ("force_state_fixed_4comp", "force_state_fixed_4comp")
                #     ],
                #     parameters=[
                #         {"boot_mass": data['force']['boot_mass']},
                #         {"boot_cdg": data['force']['boot_cdg']},
                #         {"boot_compensation": data['force']['boot_compensation']},
                #         {"fixed_frame_noise_threshold": data['force']['fixed_frame_noise_threshold']}
                #     ]
                # ),

                # ComposableNode(
                #     package='pr_modelling',
                #     node_plugin='pr_modelling::ForwardKinematics',
                #     node_name='for_kin',
                #     remappings=[
                #         ("joint_position", "joint_position"),
                #         ("x_coord", "x_coord"),
                #     ],
                #     parameters=[
                #         {"robot_config_params": data['config_params']['geometry']},
                #         {"initial_position": data['general']['init_x']},
                #         {"tol": data['general']['dir_kin']['tol']},
                #         {"iter": data['general']['dir_kin']['iter']},
                #     ]
                # ),

                # ComposableNode(
                #     package='pr_modelling',
                #     node_plugin='pr_modelling::ForwardJacobian',
                #     node_name='for_jac_med',
                #     remappings=[
                #         ("x_coord", "x_coord"),
                #         ("for_jac_det", "for_jac_det_med"),
                #         ("for_jac", "for_jac"),
                #         ("for_jac_invT","for_jac_invT")
                #     ],
                #     parameters=[
                #         {"robot_config_params": data['config_params']['geometry']},
                #     ]
                # ),
                # ComposableNode(
                #     package='pr_aux',
                #     node_plugin='pr_aux::MatrixMult',
                #     node_name='multiplier',
                #     remappings=[
                #         ("input_vector", "force_state_fixed_4comp"),
                #         ("input_matrix", "for_jac_invT"),
                #         ("output_vector", "force_joints")
                #     ],
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
                        {"gearbox_mult":  data['general']['encoder_gearbox']},
                    ],
                    extra_arguments=[{'use_intra_process_comms': intra_bool}]
                ),      
                # ComposableNode(
                #     package='pr_topic_forwarding',
                #     node_plugin='pr_topic_forwarding::TopicForwarding',
                #     node_name='topic_forwarding',
                #     # remappings=[
                #     #     ("joint_position", "joint_position")
                #     # ],
                #     # parameters=[
                #     #     {"ts_ms": data['general']['ts']*1000},
                #     #     {"initial_position": data['general']['init_q']},
                #     #     {"gearbox_mult":  data['general']['encoder_gearbox']},
                #     # ],
                #     # extra_arguments=[{'use_intra_process_comms': intra_bool}]
                # ),          
            ],
            output='screen',
    )

    return launch.LaunchDescription([pr_pid])