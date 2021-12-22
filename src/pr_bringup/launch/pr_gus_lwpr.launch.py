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
        'pr_gus.yaml'
    )

    lwpr_params_file = os.path.join(
        get_package_share_directory('pr_bringup'),
        'config',
        'pr_lwpr.yaml'
    )

    controller_yaml_file = open(controller_params_file)
    controller_params = yaml.load(controller_yaml_file)

    robot = controller_params['robot']['robot_name']
    robot_config = controller_params['robot']['config']

    robot_yaml_file = open(robot_parameters_file)
    pr_params = yaml.load(robot_yaml_file)    

    pr_config_params = pr_params[robot]['config'][robot_config]
    
    ref_file = controller_params['ref_path']['q']

    lwpr_yaml_file = open(lwpr_params_file)
    lwpr_params = yaml.load(lwpr_yaml_file)
    lwpr_params_fwd = lwpr_params['fwd']
    lwpr_params_inv = lwpr_params['inv']
    
    with open(ref_file, 'r') as f:
        first_reference = fromstring(f.readline(), dtype=float, sep=" ").tolist()
    
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
                        {"ts": controller_params['ts']},
                        {"initial_position": first_reference},
                        {"initial_reference": first_reference}
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
                        {"initial_value": first_reference},
                        {"ts": controller_params['ts']}
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
                        {"initial_value": first_reference},
                        {"ts": controller_params['ts']},
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
                        {"initD": lwpr_params_fwd['initD']},
                        {"initAlpha": lwpr_params_fwd['initAlpha']},
                        {"penalty": lwpr_params_fwd['penalty']},
                        {"initLambda": lwpr_params_fwd['initLambda']},
                        {"finalLambda": lwpr_params_fwd['finalLambda']},
                        {"activateLearning": lwpr_params_fwd['activateLearning']},
                        {"activatePrediction": lwpr_params_fwd['activatePrediction']},
                        {"loadModel": lwpr_params_fwd['loadModel']},
                        {"saveModel": lwpr_params_fwd['saveModel']}
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
                        {"initD": lwpr_params_inv['initD']},
                        {"initAlpha": lwpr_params_inv['initAlpha']},
                        {"penalty": lwpr_params_inv['penalty']},
                        {"initLambda": lwpr_params_inv['initLambda']},
                        {"finalLambda": lwpr_params_inv['finalLambda']},
                        {"activateLearning": lwpr_params_inv['activateLearning']},
                        {"activatePrediction": lwpr_params_inv['activatePrediction']},
                        {"loadModel": lwpr_params_inv['loadModel']},
                        {"saveModel": lwpr_params_inv['saveModel']},
                        {"ts": controller_params['ts']}
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
                        {"ref_path": ref_file},
                        {"is_cart": False},
                        {"robot_config_params": pr_config_params}
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
                        {"initial_position": first_reference}
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