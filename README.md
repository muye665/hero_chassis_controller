# hero_chassis_controller

## Overview

This is the practice answer(see Chinese [answer](doc/answer.md)) as a DynamicX reserve member

**Keywords:** RoboMaster, ROS, ros_control, tf

### License

The source code is released under a [BSD 3-Clause license](LICENSE).

**Author: QiayuanLiao<br />
Affiliation: [Dynamicx]()<br />
Maintainer: QiayuanLiao, liaoqiayuan@gmail.com**

The simple_chassis_controller package has been tested under [ROS] Noetic on respectively 18.04 and 20.04. This is
research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- [rm_description](https://github.com/YoujianWu/rm_description_for_task.git)
- controller_interface
- forward_command_controller
- hardware_interface
- pluginlib
- control_toolbox
- tf
- nav_msgs
- geometry_msgs

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package
using

	cd catkin_workspace/src
	git clone git@github.com:muye665/hero_chassis_controller.git
	# git clone https://github.com/muye665/hero_chassis_controller.git
	cd ../
	rosdep install --from-paths . --ignore-src
	catkin build

## Usage

Run the simulation and controller with:

	roslaunch hero_chassis_controller load_controller.launch

## Config files

Config file config

* **controllers.yaml**  Params of hero_chassis_controller and joint_state_controller.
* **pid.yaml** Params of chassis wheel's PID.

## Launch files

* **load_controller.launch:** Use keyboard to control hero chassis.

## Bugs & Feature Requests

Please report bugs and request features using
the [Issue Tracker](https://github.com/muye665/hero_chassis_controller/issues)

## Reference

**[simple_chassis_controller](https://github.com/YoujianWu/simple_chassis_controller/tree/master)**

