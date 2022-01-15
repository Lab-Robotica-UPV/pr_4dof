import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

import os

from numpy import fromstring, pi
import yaml

def generate_launch_description():

    """Generate launch description with multiple components."""

    # Load data dictionary
    import sys
    sys.path.append('src/pr_bringup/launch')
    from load_data import data

    controller_params = data['pdg_pid']

    pr_pdg = ComposableNodeContainer(
            node_name='pr_container',
            node_namespace='',
            package='rclcpp_components',
            node_executable='component_container',
            composable_node_descriptions=[
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
                        ("end_flag", "end_flag"),
                        ("joint_position", "joint_position")
                    ],
                    parameters=[
                        {"ref_path": data['general']['ref_path']['q']},
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

                ComposableNode(
                    package='pr_modelling',
                    node_plugin='pr_modelling::InverseKinematics',
                    node_name='inv_kin',
                    remappings=[
                        ("x_coord", "x_coord"),
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
                        ("x_coord", "x_coord"),
                        ("q_sol", "q_sol"),
                        ("dep_jac", "dep_jac")
                    ],
                    parameters=[
                        {"robot_config_params": data['config_params']['geometry']}
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
                        ("x_coord", "x_coord"),
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
                        ("ref_pose", "ref_pose"),
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
                    package='pr_sensors_actuators',
                    node_plugin='pr_sensors_actuators::EncodersSimulink',
                    node_name='position_sensor',
                    remappings=[
                        ("joint_position", "joint_position"),
                        ("posicion_sim", "posicion_sim"),
                    ],
                    parameters=[
                        {"ts_ms": data['general']['ts']*1000},
                        {"initial_position": data['general']['init_q']},
                    ]
                ),
                ComposableNode(
                    package='pr_sensors_actuators',
                    node_plugin='pr_sensors_actuators::MotorsSimulink',
                    node_name='actuators',
                    remappings=[
                        ("control_action", "control_action"),
                        ("voltaje_sim", "voltaje_sim"),
                    ],
                    parameters=[
                        {"vp_conversion": controller_params['vp_conversion']},
                        {"max_v": data['general']['v_sat']},
                    ]
                ),
                ComposableNode(
                    package='pr_modelling',
                    node_plugin='pr_modelling::StatePublisher',
                    node_name='state_publisher',
                    remappings=[
                        ("joint_position", "joint_position"),
                        ("joint_velocity", "joint_velocity"),
                        ("pr_state", "pr_state"),
                    ],
                ),
            ],
            output='screen',
    )

    return launch.LaunchDescription([pr_pdg])