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

    controller_yaml_file = open(controller_params_file)
    controller_params = yaml.load(controller_yaml_file)

    robot = controller_params['robot']['robot_name']
    robot_config = controller_params['robot']['config']

    robot_yaml_file = open(robot_parameters_file)
    pr_params = yaml.load(robot_yaml_file)

    mocap_yaml_file = open(mocap_config)
    mocap_params = yaml.load(mocap_yaml_file)

    force_yaml_file = open(force_file)
    force_params = yaml.load(force_yaml_file)   

    pr_config_params = pr_params[robot]['config'][robot_config]
    pr_physical_properties =  pr_params[robot]['physical_properties']
    
    ref_file_q = controller_params['ref_path']['q']
    ref_file_x = controller_params['ref_path']['x']

    ref_file_F = force_params['ref_force_path']

    with open(ref_file_x, 'r') as f:
        first_reference_x = fromstring(f.readline(), dtype=float, sep=" ").tolist()
    
    with open(ref_file_q, 'r') as f:
        first_reference_q = fromstring(f.readline(), dtype=float, sep=" ").tolist()
    
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
                    package='pr_dmp',
                    node_plugin='pr_dmp::DmpRefGen',
                    node_name='dmp_ref_gen',
                    remappings=[
                        ("joint_position", "joint_position"),
                        ("dmp_ref_gen_q", "ref_pose"),
                        ("dmp_ref_gen_x", "ref_pose_x"),
                        ("dmp_force_accel", "adm_force"),
                        ("dmp_force_vel", "dmp_force_vel"),
                        ("gka_traj","ref_force")
                    ],
                    parameters=[
                        {"ts": controller_params['ts']},
                        {"ref_path": ref_file_x},
                        {"gka_path": ref_file_F},
                        {"n_basis_functions": 200},
                        {"save_directory": ""},
                        {"isCart": True},
                        {"calcCart": True},
                        {"ref_x_init": first_reference_x},
                        {"robot_config_params": pr_config_params},
                        {"damping_coefficient": [894.0, 894.0, 89.4, 89.4]},
                        {"spring_constant": [250.0, 500.0, 25.0, 25.0]},
                        {"mass": [200.0, 200.0, 20.0, 20.0]},
                        {"speed": 1.0},
                        {"gain_slowdown": 0.1}
                    ]
                ),
                ComposableNode(
                    package='pr_dmp',
                    node_plugin='pr_dmp::AdmittanceForce',
                    node_name='dmp_admittance_force',
                    remappings=[
                        ("ref_force", "ref_force"),
                        ("force_state", "force_state_fixed"),
                        ("adm_force", "adm_force")
                    ],
                    parameters=[
                        {"K_adm": [1.0, 1.0, 1.0, 1.0]},
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

    # ros2_bag = launch.actions.ExecuteProcess(
    #     cmd=['ros2', 'bag', 'record', '-a'],
    #     output = 'screen'
    # )

    return launch.LaunchDescription([pr_dmp])