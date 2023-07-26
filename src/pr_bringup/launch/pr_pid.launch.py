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

    pr_pid = ComposableNodeContainer(
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
                    package='pr_ref_gen',
                    node_plugin='pr_ref_gen::RefPose',
                    node_name='ref_pose_gen',
                    remappings=[
                        ("ref_pose", "ref_pose"),
                        ("ref_pose_x", "useless_topic"),
                        ("end_flag", "end_flag"),
                        ("joint_position", "joint_position"),
                        ("external_stop", "joy_stop")
                    ],
                    parameters=[
                        {"ref_path": data['general']['ref_path']['q']},
                        {"is_cart": False},
                        {"robot_config_params": data['config_params']['geometry']}
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
                        {"ki_gain": controller_params['controller']['ki']},
                        {"vp_conversion": controller_params['vp_conversion']},
                        {"max_v": data['general']['robot']['v_sat']}
                    ]
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

                #   ComposableNode(
                #       package='pr_mocap',
                #       node_plugin='pr_mocap::PRXMocapRecorder',
                #       node_name='ref_x_mocap_recorder',
                #       remappings=[
                #           ("end_flag", "end_flag"),
                #           ("joint_position", "joint_position")
                #       ],
                #       parameters=[
                #           {"filename": datetime.now().strftime("%Y_%m_%d-%H_%M_%S") + "_Carles_SegFuerza"}
                #       ]
                #   ),

                ComposableNode(
                    package='pr_sensors_actuators',
                    node_plugin='pr_sensors_actuators::ForceSensor',
                    node_name='force_sensor',
                    remappings=[
                        ("joint_position", "joint_position"),
                        ("force_state", "force_state"),
                        ("force_state_sync", "force_state_sync"),
                        ("force_state_std", "force_state_std")
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
                        ("x_coord", "x_coord"),
                        ("force_state_fixed", "force_state_fixed"),
                        ("force_state_fixed_4comp", "force_state_fixed_4comp")
                    ],
                    parameters=[
                        {"boot_mass": data['force']['boot_mass']},
                        {"boot_cdg": data['force']['boot_cdg']},
                        {"boot_compensation": data['force']['boot_compensation']},
                        {"fixed_frame_noise_threshold": data['force']['fixed_frame_noise_threshold']}
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
                    package='pr_modelling',
                    node_plugin='pr_modelling::ForwardKinematics',
                    node_name='for_kin',
                    remappings=[
                        ("joint_position", "joint_position"),
                        ("x_coord", "x_coord"),
                    ],
                    parameters=[
                        {"robot_config_params": data['config_params']['geometry']},
                        {"initial_position": data['general']['init_x']},
                        {"tol": data['general']['dir_kin']['tol']},
                        {"iter": data['general']['dir_kin']['iter']},
                    ]
                ),

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
                    package='pr_joy',
                    node_plugin='pr_joy::JoyStop',
                    node_name='joy_stop',
                    remappings=[
                        ("joy", "joy"),
                        ("joy_stop", "joy_stop")
                    ],
                ),   
                
                # ComposableNode(
                #     package='pr_topic_forwarding',
                #     node_plugin='pr_topic_forwarding::ArrayToQuaternion',
                #     node_name='pos_q_std',
                #     remappings=[
                #         ("array_topic", "joint_position"),
                #         ("quaternion_topic", "pos_q_std")
                #     ],
                # ),  

                ComposableNode(
                    package='pr_topic_forwarding',
                    node_plugin='pr_topic_forwarding::ArrayToQuaternion',
                    node_name='ref_x_std',
                    remappings=[
                        ("array_topic", "ref_x"),
                        ("quaternion_topic", "ref_x_std")
                    ],
                ),  

                ComposableNode(
                    package='pr_topic_forwarding',
                    node_plugin='pr_topic_forwarding::ArrayToQuaternion',
                    node_name='pos_x_std',
                    remappings=[
                        ("array_topic", "x_coord"),
                        ("quaternion_topic", "pos_x_std")
                    ],
                ),  

                 ComposableNode(
                    package='pr_topic_forwarding',
                    node_plugin='pr_topic_forwarding::ArrayToQuaternion',
                    node_name='ref_force_std',
                    remappings=[
                        ("array_topic", "ref_force"),
                        ("quaternion_topic", "ref_force_std")
                    ],
                ),  

                ComposableNode(
                    package='pr_topic_forwarding',
                    node_plugin='pr_topic_forwarding::ForceStateToWrench',
                    node_name='force_std',
                    remappings=[
                        ("force_topic", "force_state_fixed"),
                        ("wrench_topic", "force_std")
                    ],
                ),

                ComposableNode(
                    package='pr_modelling',
                    node_plugin='pr_modelling::ForwardKinematics',
                    node_name='for_kin',
                    remappings=[
                        ("joint_position", "ref_pose"),
                        ("x_coord", "ref_x"),
                    ],
                    parameters=[
                        {"robot_config_params": data['config_params']['geometry']},
                        {"initial_position": data['general']['init_x']},
                        {"tol": data['general']['dir_kin']['tol']},
                        {"iter": data['general']['dir_kin']['iter']},
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
                        {"gearbox_mult":  data['general']['encoder_gearbox']},
                    ]
                ),            
            ],
            output='screen',
    )

    return launch.LaunchDescription([pr_pid])