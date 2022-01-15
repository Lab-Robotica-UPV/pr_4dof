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
# First references
with open(general_params['ref_path']['x'], 'r') as f:
    init_x = fromstring(f.readline(), dtype=float, sep=" ").tolist()
with open(general_params['ref_path']['q'], 'r') as f:
    init_q = fromstring(f.readline(), dtype=float, sep=" ").tolist()
general_params['init_x'] = init_x
general_params['init_q'] = init_q

data['general'] = general_params

# Config params
config_params = load_params('config_params.yaml')
robot_name = data['general']['robot']['robot_name']
configuration = data['general']['robot']['config']
data['config_params'] = {}
data['config_params']['geometry'] = config_params[robot_name]['geometry'][configuration]
data['config_params']['physical_properties'] = config_params[robot_name]['physical_properties']
data['config_params']['q_lim'] = config_params[robot_name]['q_lim']

# Force
force_params = load_params('force.yaml')
data['force'] = force_params

# Gus
gus_params = load_params('gus.yaml')
data['gus'] = gus_params

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
data['pdg_pid'] = pdg_pid_params

# Sing
sing_params = load_params('sing.yaml')
data['sing'] = {}
data['sing']['ots'] = sing_params['ots']
data['sing']['lmin_Ang_OTS'] = sing_params['lmin_Ang_OTS'][configuration]
data['sing']['lmin_FJac'] = sing_params['lmin_FJac'][configuration]
data['sing']['t_activation_releaser'] = sing_params['t_activation_releaser']

# Dmp
dmp_params = load_params('dmp.yaml')
data['dmp'] = dmp_params

# import json
# print(json.dumps(data, indent=4))