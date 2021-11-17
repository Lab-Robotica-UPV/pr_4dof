import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

import os

from numpy import fromstring, pi
import yaml

def generate_launch_description():

    """Generate launch description with multiple components."""

    robot_parameters_file = os.path.join(
        get_package_share_directory('pr_bringup'),
        'config',
        'pr_config_params.yaml'
    )

    controller_params_file = os.path.join(
        get_package_share_directory('pr_bringup'),
        'config',
        'pr_pdg_pid.yaml'
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

    robot_yaml_file = open(robot_parameters_file)
    pr_params = yaml.load(robot_yaml_file)

    controller_yaml_file = open(controller_params_file)
    controller_params = yaml.load(controller_yaml_file)

    mocap_yaml_file = open(mocap_config)
    mocap_params = yaml.load(mocap_yaml_file)

    force_yaml_file = open(force_file)
    force_params = yaml.load(force_yaml_file)

    robot = controller_params['robot']['robot_name']
    robot_config = controller_params['robot']['config']    

    pr_config_params = pr_params[robot]['config'][robot_config]
    pr_physical_properties =  pr_params[robot]['physical_properties']

    admittance_params = force_params['admittance_params']
    ref_file_F = force_params['ref_force_path']

    ref_file_q = controller_params['ref_path']['q']
    ref_file_x = controller_params['ref_path']['x']

    with open(ref_file_q, 'r') as f:
        first_reference_q = fromstring(f.readline(), dtype=float, sep=" ").tolist()
    
    with open(ref_file_x, 'r') as f:
        first_reference_x = fromstring(f.readline(), dtype=float, sep=" ").tolist()

    pr_pdg = ComposableNodeContainer(
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
                        {"vp_conversion": controller_params['actuators']['vp_conversion'][1]},
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
                        {"vp_conversion": controller_params['actuators']['vp_conversion'][2]},
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
                        {"vp_conversion": controller_params['actuators']['vp_conversion'][3]},
                        {"max_v": controller_params['actuators']['v_sat']}
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
                        {"calibration": force_params['calibration']},
                        {"noise_threshold": force_params['noise_threshold']}
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
                        {"boot_mass": force_params['boot_mass']},
                        {"boot_cdg": force_params['boot_cdg']},
                        {"boot_compensation": force_params['boot_compensation']},
                        {"fixed_frame_noise_threshold": force_params['fixed_frame_noise_threshold']}
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
                    node_plugin='pr_modelling::InverseKinematics',
                    node_name='inv_kin',
                    remappings=[
                        ("x_coord", "x_mocap_sync"),
                        ("q_sol", "q_sol"),
                    ],
                    parameters=[
                        {"robot_config_params": pr_config_params},
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
                        {"robot_config_params": pr_config_params}
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
                        {"p11": pr_physical_properties['p11']},
                        {"p12": pr_physical_properties['p12']},
                        {"p21": pr_physical_properties['p21']},
                        {"p22": pr_physical_properties['p22']},
                        {"p31": pr_physical_properties['p31']},
                        {"p32": pr_physical_properties['p32']},
                        {"p41": pr_physical_properties['p41']},
                        {"p42": pr_physical_properties['p42']},
                        {"pm":  pr_physical_properties['pm']},
                    ]
                ),

                ComposableNode(
                    package='pr_controllers',
                    node_plugin='pr_controllers::PDGController',
                    node_name='controller',
                    remappings=[
                        ("ref_pose", "ref_sum"),
                        ("joint_position", "joint_position"),
                        ("joint_velocity", "joint_velocity"),
                        ("q_grav", "q_grav")
                    ],
                    parameters=[
                        {"kp_gain": controller_params['controller']['kp']},
                        {"kv_gain": controller_params['controller']['kv']},
                        {"initial_position": first_reference_q},
                        {"initial_reference": first_reference_q}
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

    return launch.LaunchDescription([pr_pdg])