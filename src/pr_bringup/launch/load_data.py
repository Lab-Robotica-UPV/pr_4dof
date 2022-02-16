import os

from ament_index_python.packages import get_package_share_directory

from numpy import fromstring

import yaml

def load_params(filename):
    parameters_file = os.path.join(
        get_package_share_directory('pr_bringup'),
        'config',
        filename
    )
    yaml_file = open(parameters_file)
    params = yaml.load(yaml_file)
    return params

# Dictionary with all data
data = dict()

# General data
general_params = load_params('general.yaml')
data['general'] = {}
data['general']['ts'] = general_params['ts']
data['general']['robot'] = general_params['robot']
robot_name = data['general']['robot']['robot_name']
configuration = data['general']['robot']['config']
data['general']['encoder_gearbox'] = general_params[robot_name]['encoder_gearbox']
data['general']['ref_path'] = general_params[robot_name]['ref_path']
data['general']['dir_kin'] = general_params['dir_kin']
data['general']['displayer'] = general_params['displayer']

# First references
with open(general_params[robot_name]['ref_path']['x'], 'r') as f:
    init_x = fromstring(f.readline(), dtype=float, sep=" ").tolist()
    nonempty_lines = [line.strip("\n") for line in f if line != "\n"]
    data['general']['num_samples'] = len(nonempty_lines) + 1
with open(general_params[robot_name]['ref_path']['q'], 'r') as f:
    init_q = fromstring(f.readline(), dtype=float, sep=" ").tolist()
data['general']['init_x'] = init_x
data['general']['init_q'] = init_q


# Config params
config_params = load_params('config_params.yaml')
data['config_params'] = {}
data['config_params']['geometry'] = config_params[robot_name]['geometry'][configuration]
data['config_params']['physical_properties'] = config_params[robot_name]['physical_properties']
data['config_params']['q_lim'] = config_params[robot_name]['q_lim']


# Force
force_params = load_params('force.yaml')
data['force'] = force_params

# Gus
gus_params = load_params('gus.yaml')
data['gus'] = {}
data['gus']['vp_conversion'] = gus_params[robot_name]['vp_conversion']
data['gus']['controller'] = gus_params[robot_name]['controller']

# ILC
ilc_params = load_params('ilc.yaml')
data['ilc'] = ilc_params

# LWPR
lwpr_params = load_params('lwpr.yaml')
data['lwpr'] = lwpr_params

# Mocap server
mocap_server_params = load_params('mocap_server.yaml')
data['mocap_server'] = {}
data['mocap_server']['server_address'] = mocap_server_params['server_address']
data['mocap_server']['server_command_port'] = mocap_server_params['server_command_port']
data['mocap_server']['server_data_port'] = mocap_server_params['server_data_port']
data['mocap_server']['marker_names'] = mocap_server_params['marker_names'][robot_name]

# PDG/PID
pdg_pid_params = load_params('pdg_pid.yaml')
data['pdg_pid'] = {}
data['pdg_pid']['vp_conversion'] = pdg_pid_params[robot_name]['vp_conversion']
data['pdg_pid']['controller'] = pdg_pid_params[robot_name]['controller']

# Sing
sing_params = load_params('sing.yaml')
data['sing'] = {}
data['sing']['ots'] = sing_params['ots']
data['sing']['lmin_Ang_OTS'] = sing_params[robot_name]['lmin_Ang_OTS'][configuration]
data['sing']['lmin_FJac'] = sing_params[robot_name]['lmin_FJac'][configuration]
data['sing']['t_activation_releaser'] = sing_params['t_activation_releaser']

# Dmp
dmp_params = load_params('dmp.yaml')
data['dmp'] = dmp_params

# import json
# print(json.dumps(data, indent=4))