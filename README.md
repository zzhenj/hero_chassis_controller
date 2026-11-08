# hero_chassis_controller

## Overview

This is a ros_control controller for the RoboMaster hero robot with mecanum wheels. It receives velocity commands
on `/cmd_vel`, computes the desired angular velocity of each wheel through **inverse kinematics**, and drives each
wheel through an independent **PID** loop using `control_toolbox`. The chassis velocity is reconstructed from the measured
wheel velocities via **forward kinematics**, and published as `nav_msgs/Odometry` on `/odom` together with
an **"odom" → "base_link"** tf transform. An optional **global velocity mode** accepts `/cmd_vel` in the world frame
and transforms it into the chassis frame via tf.

This is controller example/template for the assignment (see Chinese [requirement](../simple_chassis_controller/doc/requirement.md)).

**Keywords:** RoboMaster, ROS, ros_control, mecanum wheel, odometry

### License

The source code is released under a [BSD 3-Clause license](LICENSE).

**Author: Zane Ji<br />
Maintainer: Zane Ji, k.jzz00625@gmail.com**

The hero_chassis_controller package has been tested under [ROS] Noetic on 20.04. This is research code, expect that
it changes often and any fitness for a particular purpose is disclaimed.

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- [rm_description_for_task](https://github.com/Bluefairy-ljy/rm_description_for_task) (URDF of the hero robot),
- [rm_gazebo](https://github.com/gdut-dynamic-x/rm_control) (Gazebo robot hardware simulation),
- ros_control
    - controller_interface
    - controller_manager
    - hardware_interface
- control_toolbox
- realtime_tools
- pluginlib
- tf
- tf2_ros
- tf2_geometry_msgs
- nav_msgs
- geometry_msgs

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package
using

	cd catkin_workspace/src
	git clone git@github.com:zzhenj/hero_chassis_controller.git
    # git clone https://github.com/zzhenj/hero_chassis_controller.git
	cd ../
	rosdep install --from-paths . --ignore-src
	catkin build

## Usage

Run the simulation and controller with:

	roslaunch hero_chassis_controller hero_chassis_controller.launch

Then send commands on `/cmd_vel`:

	# forward
	rostopic pub -r 10 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.5}}'
	# backward
	rostopic pub -r 10 /cmd_vel geometry_msgs/Twist '{linear: {x: -0.5}}'
	# strafe left
	rostopic pub -r 10 /cmd_vel geometry_msgs/Twist '{linear: {y: 0.5}}'
	# rotate counter-clockwise
	rostopic pub -r 10 /cmd_vel geometry_msgs/Twist '{angular: {z: 0.5}}'
	# stop
	rostopic pub -1 /cmd_vel geometry_msgs/Twist '{}'

Check the odometry:

	rostopic echo /odom
	rosrun tf tf_echo odom base_link

Tune the PID gains online:

	rqt_plot /joint_states/velocity[0] /joint_states/velocity[1] \
	         /joint_states/velocity[2] /joint_states/velocity[3]
	rosrun rqt_reconfigure rqt_reconfigure

## Config files

Config file config

* **hero_chassis_controller.yaml**  Params of hero_chassis_controller and joint_state_controller.

* **wheel_radius** Wheel radius in meter, obtained from `rm_description_for_task` (`mecanum_wheel.urdf.xacro`).

* **track_width_x / track_width_y** Distance between the front/back and left/right wheel centres in meter, obtained
  from `chassis.urdf.xacro` (`wheel_track` / `wheel_base`).

* **velocity_mode** Velocity command frame. `chassis` (default) treats `/cmd_vel` as a chassis frame command,
  `global` treats it as a world frame command which is transformed to the chassis frame via tf.

* **world_frame** World frame used in `global` mode, e.g. `odom` or `map`.

* **wheels/<wheel>/joint** Joint name of each wheel.

* **wheels/<wheel>/pid** PID params (`p`, `i`, `d`, `i_clamp_max`, `i_clamp_min`, `antiwindup`) of each wheel,
  adjustable at runtime through `dynamic_reconfigure`, i.e. `rqt_reconfigure`.

* **joint_state_controller** Publishes `sensor_msgs/JointState` on `/joint_states`, needed by `rqt_plot` and
  `robot_state_publisher`.

## Launch files

* **hero_chassis_controller.launch:** Hero chassis only simulation, load the controller config and spawn
  `joint_state_controller` and `hero_chassis_controller`.

## Bugs & Feature Requests

Please report bugs and request features using
the [Issue Tracker](https://github.com/zzhenj/hero_chassis_controller/issues)
.

[ROS]: http://www.ros.org