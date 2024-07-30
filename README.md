<h1 align="center">
  Controllers for the 3UPS+RPU Parallel Robot
  <br>
</h1>

<p align="center">
  This repository provides the position/force control laws developed in C++ over the middleware <a href="https://docs.ros.org/en/eloquent/index.html">ROS2 Eloquent</a> for controlling a prototype of a Parallel Robot (PR) for knee rehabilitation. The PR for knee rehabilitation is a 4 Degrees of Freedom (DOF) mechanism designed and built at Universitat Politècnica de València (<a href="https://www.upv.es/">UPV</a>), Spain.
</p>

<p align="center">
  Projects Websites:
  <br>
  <a href="https://imbio3r.ai2.upv.es/index.htm">IMBiO3R</a>
  ·
  <a href="https://roboprop.ai2.upv.es/">ROBOPROP</a>
</p>

## Overview
The packages developed are organized as follow:

* [**pr_aux**](/src/pr_aux) - A series of utilities for adding, derivating and filtering the custom messages.
* [**pr_biomech**](/src/pr_biomech) - Forward and Inverse kinematic and dynamic solution for a biomechanic model of the right lower limb.
* [**pr_bringup**](/src/pr_bringup) - A series of lauchs files for executing the position/force controllers for the 3UPS+RPU PR.
* [**pr_controllers**](/src/pr_controllers) - Packages for the different control laws without connection to the inputs (encoders) and outputs(actuators).
* [**pr_dmp**](/src/pr_dmp) - Package for path generation based on Dynamics Primitive Movements (DMPs).
* [**pr_ilc**](/src/pr_ilc) - Package for testing the Iterative Learning Control.
* [**pr_joy**](/src/pr_joy) - Package for connecting the joystick package to the compasable node. **The lauch files are not using**
* [**pr_lib**](/src/pr_lib) - A set of utilities for solving the kinematic and dyanmic problem of the 3UPS+RPU PR and detecting Type II singularities in the same 4-DOF PR.
* [**pr_lib_biomech**](/src/pr_lib_biomech) - A set of utilities for solving the Forward and Inverse kinematic and dynamic problems for a biomechanic model of the right lower limb.
* [**pr_mocap**](/src/pr_mocap) - A set of utilities for connecting to OptiTrack 3D tracking system (3DTS) and retriving the real position and orientation of the 3UPS+RPU PR. There is the option to start the recording in the Optitrack 3DTS.
* [**pr_modelling**](/src/pr_modelling) - Provides the packeges developed for calculating the jacobian matrices for the 3UPS+RPU PR. It includes the admittance modelling with/without the stiffness component, the Output Twist Screws calculation require for singularity detection and the adaptive law developed by **Lotfy**.
* [**pr_msgs**](/src/pr_msgs) - A set of custom messages developed for handling the signals of the 4-DOF PR.
* [**pr_ref_gen**](/src/pr_ref_gen) - Packages that read a txt file with the position path (in configuration or joint space) and retrieve each sample when the encoders read a new sample.
* [**pr_sensors_actuators**](/src/pr_sensors_actuators) - Packages that handle the encoders reading, the control actions apllied to the actuators and the 6-DOF Force sensor.

## Getting started
1. In a new terminal, initialize the ROS 2 enviroment by runnign the following lines:

```bash
# Change to ROS2 workspace directory
ros2 cd /home/paralelo4dofnew/ros2_eloquent_ws/pr_4dof/
# Source the ROS2 setup script
ros2 source /opt/ros/ros2_eloquent/setup.bash
# Install the PR_4DOF packages
. install/setup.bash
```
> [!NOTE]
> IN THE INDUSTRIAL PC AT ROBOTICS LAB EACH TERMINAL INITIALIZE ROS2 AND PR_4DOF PACKAGES BECASUSE THE PREVIOUS LINES ARE ADDED TO `.bashrc`. It is recommended to add the previous commands to your `.bashrc`.  

2. The package `pr_bringup` manages the launch files for each controller developed for the 4-DOF PR.
This is an example of how to launch a Linear Algebra model-based controller:

```bash
ros2 launch pr_bringup pr_gus.launch.py
```
The next section present a complete list of controllers ready to use.

## List of controllers ready to use (Available lauch files)
The lauch files avilable for executing the controllers are organized as:
* **Position control**
    * `pr_ref_auto_pid.launch.py` - PID controller that move the 4-DOF PR from the actual pose (*The actual pose is measured by the Optitrack System*) to the initial sample of the predefined path in joint space.
    * `pr_pid_joystick.launch.py` - PID controller for controlling the 4-DOF PR according to a predefined path in joint space. *The joystick is enable to allow an external stop handled by the user*
    * `pr_pid.launch.py` - PID controller for controlling the 4-DOF PR according to a predefined path in joint space.
    * `pr_pdg.launch.py` - PD with gravitational compensation controller for controlling the 4-DOF PR according to a predefined path in configuration or joint space.
* **Force/Position control**
    * `pr_force_pid_pos_fixedframe_bootcomp_ref_sat.launch.py` - Force/position controller (Admittance control) for the 4-DOF PR that follows a predefined pose and force reference path.
    * `pr_force_pid_static_fixedframe_bootcomp_auto_sat.launch.py` - Admittance control that set the 4-DOF PR in compliant mode from the actual pose. *The pose and force refenrence are automatic generated based on the measurements taken from the Optitrack System*.
    * `pr_dmp_reversible_flim_pid.launch.py` - A reversible DMP controller that allows to control the execution of the trajectory according to the force excerted by the patient. **The controller is developed for propioception test. Thus, the trajectory requires a first part for go and back to the final pose, the second part is controlled by the force's users.** *The sample for activating the force control in the second part is automatic detected by the controller.* *The joystick is enable to allow an external stop handled by the user*

* **Adaptive control**
    * `pr_pdg_adaptive_rbfg.launch.py` - Initial adptive controller developed to test the control law defined by Lotfy using Radial Basis Fuction Neural Network.