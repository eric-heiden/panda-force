# Panda High-Level Control/Sensing Interface
This repository contains some force/torque sensing and control experiments for the Franka Emika Panda robot arm, which is based on the libfranka C++ API.

This code is part of the real-robot experiments performed at USC for the [DiSECt project on robotic cutting](https://diff-cutting-sim.github.io/).

## ZMQ Control Server

The core component is the control server written in C++, which runs on the real-time kernel and accepts commands/queries over a ZeroMQ interface from a client running on another machine.

### Back-end (C++ on real-time kernel)
Among others, the [control server](experiments/zmq_control_server.cpp) has the following capabilities, which are exposed through an easy-to-use Python interface:

| Command | Description |
| --- | --- |
| `connect` | Connect to the robot |
| `disconnect` | Disconnect from the robot |
| `get_state` | Get the current state of the robot |
| `error_recovery` | Recover from any errors that may have been encountered in a previous execution |
| `set_joint_impedance` | Set the joint impedance parameters (list of 7 stiffness values) |
| `set_collision_behavior` | Set the collision behavior of the robot (list of 7 `lower_torque_thresholds` values, list of 7 `upper_torque_thresholds` values, list of 6 `lower_force_thresholds` values, list of 6 `upper_force_thresholds` values) |
| `record` | Record [robot states](https://frankaemika.github.io/libfranka/structfranka_1_1RobotState.html) (torques, q, qd, gravity, end-effector wrench, etc.) at high frequency (recordings are saved to JSON files and sent over ZMQ when the recording is stopped) |
| `retrieve` | Retrieve recorded robot states (torques, q, qd, gravity, end-effector wrench, etc.) from a previous recording based on the JSON log file name |
| `move_to_q` | Move to joint configuration with a desired speed (based on the `MotionGenerator` provided in the libfranka demos) |
| `interpolate` | Interpolate a sequence of joint positions (7D "way points") via cubic splines (it will only return the result, not execute the trajectory, for debugging purposes), considering a source and target sampling time step |
| `follow_qs` | Follow a sequence of joint positions (7D "way points") using cubic spline interpolation while recording robot states during the execution |
| `follow_cartesian_vel` | Follow a sequence of cartesian velocities (6D end-effector twist "way points") using cubic spline interpolation while recording robot states during the execution |

### Front-end (Python from anywhere)
The control server is exposed through a Python interface, which is used by the [control client](experiments/zmq_control_client.py). This makes it possible to control the robot from any machine that has access to the control server, but does not need any special real-time kernel setup.

Besides exposing the functions from the back-end, the client also offers 3D visualization and inverse kinematics functions via the [Robotics Toolbox for Python](https://github.com/petercorke/robotics-toolbox-python):


## Requirements

Modern C++ compiler with C++17 support. The C++ examples need to be executed on the real-time kernel that is connected to the Panda robot.

### Dependencies:
* Eigen 3.3+
* ZeroMQ (for the zmq control server)
* libfranka (can be built and installed from the git submodule [here](libfranka/))
* cppzmq (requires can be built and installed from the git submodule [here](cppzmq/))

### Python dependencies:
* numpy
* matplotlib
* zmq
* Robotics Toolbox for Python
```
pip3 install roboticstoolbox-python
```

## Examples

The following examples are available in `experiments/`:

| Name | Description |
| --- | --- |
| `example_cutting.py` | Example using the Python front-end to the ZMQ server for a vertical cutting motion that is computed as a Cartesian position trajectory, converted to joint positions via IK, and executed on the robot. |
|  |
| `cartesian_velocity_control.cpp` | Basic Cartesian velocity control from the libfranka-provided demos. |
| `echo_robot_state.cpp` | Records the robot states to a JSON file, adapted from the libfranka-provided demos. |
| `force_control.cpp` | Impedance-based controller that reaches a  vertical end-effector force downwards to achieve a desired mass, adapted from the libfranka-provided demos. The end-effector must be in contact with something before running this demo. |
| `cut_vertical.cpp` | Outdated C++ code for vertical cutting. |
| `cut_vertical_edc.cpp` | Outdated C++ code for vertical cutting. |