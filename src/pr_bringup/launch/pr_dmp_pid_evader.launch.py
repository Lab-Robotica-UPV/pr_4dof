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
                    node_plugin='pr_dmp::DmpRefGen',
                    node_name='dmp_ref_gen',
                    remappings=[
                        ("joint_position", "joint_position"),
                        ("dmp_ref_gen_q", "ref_pose"),
                        ("dmp_ref_gen_x", "ref_pose_x"),
                        ("dmp_force_accel", "dmp_force"),
                        ("dmp_force_vel", "dmp_force_vel"),
                        ("gka_traj","ref_force")
                    ],
                    parameters=[
                        {"ts": data['general']['ts']},
                        {"ref_path": data['general']['ref_path']['q']},
                        {"gka_path": ""},
                        {"n_basis_functions": 200},
                        {"save_directory": ""},
                        {"isCart": False},
                        {"calcCart": False},
                        {"ref_x_init": data['general']['init_x']},
                        {"robot_config_params": data['config_params']['geometry']},
                        {"damping_coefficient": data['dmp']['damping']},
                        {"spring_constant": data['dmp']['spring']},
                        {"mass": data['dmp']['mass']},
                        {"speed": data['dmp']['speed']},
                        {"gain_slowdown": data['dmp']['gain_slowdown']}
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
                        ("evader_force", "dmp_force")
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
                        {"base_inc_force_jac": data['dmp']['spring'][0]*0.0004},
                        {"base_inc_force_OTS": data['dmp']['spring'][0]*0.0004},
                        {"dec_force_jac": data['dmp']['spring'][0]*0.0004/20.0},
                        {"dec_force_OTS": data['dmp']['spring'][0]*0.0004/20.0},
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