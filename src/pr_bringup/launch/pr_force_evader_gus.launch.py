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

    robot_parameters_file = os.path.join(
        get_package_share_directory('pr_bringup'),
        'config',
        'pr_config_params.yaml'
    )

    controller_params_file = os.path.join(
        get_package_share_directory('pr_bringup'),
        'config',
        'pr_gus_sing_releaser_evader.yaml'
    )

    mocap_config = os.path.join(
        get_package_share_directory('pr_bringup'),
        'config',
        'mocap_server.yaml'
        )

    force_file = os.path.join(
        get_package_share_directory('pr_bringup'),
        'config',
        'pr_force.yaml'
    )

    controller_yaml_file = open(controller_params_file)
    controller_params = yaml.load(controller_yaml_file)

    mocap_yaml_file = open(mocap_config)
    mocap_params = yaml.load(mocap_yaml_file)

    force_yaml_file = open(force_file)
    force_params = yaml.load(force_yaml_file)

    robot = controller_params['robot']['robot_name']
    robot_config = controller_params['robot']['config']

    robot_yaml_file = open(robot_parameters_file)
    pr_params = yaml.load(robot_yaml_file)    

    admittance_params = force_params['admittance_params']
    ref_file_F = force_params['ref_force_path']

    pr_config_params = pr_params[robot]['config'][robot_config]
        
    ref_file_q = controller_params['ref_path']['evader']['q']
    ref_file_x = controller_params['ref_path']['evader']['x']

    with open(ref_file_q, 'r') as f:
        first_reference_q = fromstring(f.readline(), dtype=float, sep=" ").tolist()
    
    with open(ref_file_x, 'r') as f:
        first_reference_x = fromstring(f.readline(), dtype=float, sep=" ").tolist()

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
                        {"vp_conversion": controller_params['actuators']['vp_conversion'][0]},
                        {"max_v": controller_params['actuators']['v_sat']}
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
                        {"vp_conversion": controller_params['actuators']['vp_conversion'][0]},
                        {"max_v": controller_params['actuators']['v_sat']}
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
                        {"vp_conversion": controller_params['actuators']['vp_conversion'][0]},
                        {"max_v": controller_params['actuators']['v_sat']}
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
                        {"vp_conversion": controller_params['actuators']['vp_conversion'][0]},
                        {"max_v": controller_params['actuators']['v_sat']}
                    ]
                ),

                ComposableNode(
                    package='pr_sensors_actuators',
                    node_plugin='pr_sensors_actuators::ForceSensor',
                    node_name='force_sensor',
                    remappings=[
                        ("force_state", "force_state"),
                        ("force_state_accelstamped", "force_state_accelstamped")
                    ],
                    parameters=[
                        {"calibration": force_params['calibration']},
                        {"noise_threshold": force_params['noise_threshold']}
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
                        {"ts": controller_params['ts']},
                        {"initial_position": first_reference_q},
                        {"initial_reference": first_reference_q}
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
                        {"initial_value": first_reference_q},
                        {"ts": controller_params['ts']}
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
                        {"ref_path": ref_file_x},
                        {"is_cart": True},
                        {"robot_config_params": pr_config_params}
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
                        {"ref_path": ref_file_F},
                        {"is_cart": False},
                        {"robot_config_params": pr_config_params}
                    ]
                ),

                ComposableNode(
                    package='pr_modelling',
                    node_plugin='pr_modelling::Admittance',
                    node_name='admittance',
                    remappings=[
                        ("force_state", "force_state"),
                        ("ref_force", "ref_force"),
                        ("activation_pin", "activation_pin"),
                        ("vel_admittance", "vel_admittance"),
                        ("pos_admittance", "pos_admittance")
                    ],
                    parameters=[
                        {"mass": admittance_params['mass']},
                        {"damping": admittance_params['damping']},
                        {"stiffness": admittance_params['stiffness']},
                        {"ts": controller_params['ts']}
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
                        {"robot_config_params": pr_config_params},
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
                        {"robot_config_params": pr_config_params},
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
                        {"robot_config_params": pr_config_params},
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
                        {"robot_config_params": pr_config_params},
                        {"initial_ots": [0.0, 0.0, 1.0, 0.0, 0.0, 1.0]},
                        {"initial_position": first_reference_x},
                        {"iter_max_ots": controller_params['ots']['iter']},
                        {"tol_ots": controller_params['ots']['tol']},
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
                        {"robot_config_params": pr_config_params},
                        {"initial_ots": [0.0, 0.0, 1.0, 0.0, 0.0, 1.0]},
                        {"initial_position": first_reference_x},
                        {"iter_max_ots": controller_params['ots']['iter']},
                        {"tol_ots": controller_params['ots']['tol']},
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
                        ("ref_pose_mod", "ref_pose_mod"),
                        ("sing_pin", "activation_pin")
                    ],
                    parameters=[
                        {"robot_config_params": pr_config_params},
                        {"lmin_Ang_OTS": controller_params['sing_releaser_evader']['lmin_Ang_OTS']},
                        {"lmin_FJac": controller_params['sing_releaser_evader']['lmin_FJac']},
                        {"iter_fk": controller_params['sing_releaser_evader']['fk']['iter']},
                        {"tol_fk": controller_params['sing_releaser_evader']['fk']['tol']},
                        {"iter_OTS": controller_params['sing_releaser_evader']['ots']['iter']},
                        {"tol_OTS": controller_params['sing_releaser_evader']['ots']['tol']},
                        {"ts": controller_params['ts']}
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
                        {"server_address": mocap_params["server_address"]},
                        {"server_command_port": mocap_params["server_command_port"]},
                        {"server_data_port": mocap_params["server_data_port"]},
                        {"marker_names":  mocap_params["marker_names"][robot]},
                        {"robot_5p": robot=="robot_5p"},
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
                        {"ts_ms": controller_params['ts']*1000},
                        {"initial_position": first_reference_q}
                    ]
                ),
            ],
            output='screen',
    )

    return launch.LaunchDescription([pr_gus])