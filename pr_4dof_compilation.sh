# Execute ./pr_4dof_compilation.sh to compile all packages

colcon build --packages-select pr_msgs
. install/setup.bash
colcon build --packages-select pr_lib pr_lib_biomech
. install/setup.bash
colcon build --packages-select pr_modelling
. install/setup.bash
colcon build --packages-select pr_aux pr_bringup pr_controllers pr_ref_gen pr_rl pr_sing pr_lwpr pr_ilc pr_biomech pr_dmp
. install/setup.bash

# Esta ultima solo funciona en el ordenador de control
colcon build --packages-select pr_mocap pr_sensors_actuators
. install/setup.bash
