/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2026, Zane Ji
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

#ifndef HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H
#define HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H

#include <memory>
#include <string>

#include <control_toolbox/pid.h>
#include <controller_interface/controller.h>
#include <geometry_msgs/Twist.h>
#include <hardware_interface/joint_command_interface.h>
#include <nav_msgs/Odometry.h>
#include <realtime_tools/realtime_buffer.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <ros/time.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

namespace hero_chassis_controller
{
/**
 * @brief Mecanum wheel chassis controller for the RoboMaster hero robot.
 *
 * Subscribes to /cmd_vel, computes the desired angular velocity of each wheel
 * through inverse kinematics, and drives each wheel through an independent PID
 * loop. The chassis velocity is reconstructed from the measured wheel
 * velocities and published as nav_msgs/Odometry on /odom together with an odom
 * -> base_link tf transform.
 *
 * In "global" velocity mode the /cmd_vel command is interpreted in the world
 * frame and transformed into the chassis frame via tf.
 */
class HeroChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:
  /**
   * @brief Constructor.
   */
  HeroChassisController() = default;

  /**
   * @brief Destructor.
   */
  ~HeroChassisController() override = default;

  /**
   * @brief Initialize the controller.
   *
   * Reads the chassis geometry and PID parameters from the ROS parameter
   * server, acquires the joint handles of the four wheels, sets up the /cmd_vel
   * subscriber and the /odom publisher.
   *
   * @param effort_joint_interface The effort joint interface of the robot
   * hardware.
   * @param root_nh The root node handle, used for the /cmd_vel subscriber and
   * /odom publisher.
   * @param controller_nh The node handle of this controller, used for the
   * parameters.
   * @return true if the controller was initialized successfully, false
   * otherwise.
   */
  bool init(hardware_interface::EffortJointInterface* effort_joint_interface, ros::NodeHandle& root_nh,
            ros::NodeHandle& controller_nh) override;

  /**
   * @brief Called when the controller is started.
   *
   * Clears the command buffer and the odometry, and resets the PID controllers
   * so that no stale state is carried over from a previous run.
   *
   * @param time The current time.
   */
  void starting(const ros::Time& time) override;

  /**
   * @brief Control loop, called at the hardware update rate.
   *
   * Reads the latest /cmd_vel command, optionally transforms it into the
   * chassis frame, applies inverse kinematics, and drives the wheel PIDs.
   * Odometry and the odom -> base_link transform are updated at the end of the
   * loop.
   *
   * @param time The current time.
   * @param period The time elapsed since the last update.
   */
  void update(const ros::Time& time, const ros::Duration& period) override;

  /**
   * @brief Called when the controller is stopped.
   *
   * Sets all joint commands to zero.
   *
   * @param time The current time.
   */
  void stopping(const ros::Time& time) override;

private:
  /**
   * @brief Callback for the /cmd_vel topic.
   * @param msg The velocity command.
   */
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);

  /// @brief Subscriber for /cmd_vel.
  ros::Subscriber sub_cmd_vel_;

  /// @brief Thread-safe buffer holding the latest velocity command.
  realtime_tools::RealtimeBuffer<geometry_msgs::Twist> command_buffer_;

  /// @brief Joint handles of the four wheels, in the order lf, rf, lb, rb.
  hardware_interface::JointHandle wheel_joints_[4];

  /// @brief Wheel radius in meters.
  double wheel_radius_;

  /// @brief Distance between the front and back wheel centres in meters.
  double track_width_x_;

  /// @brief Distance between the left and right wheel centres in meters.
  double track_width_y_;

  /// @brief PID controllers of the four wheels, one per wheel.
  control_toolbox::Pid pid_controllers_[4];

  /// @brief Estimated x position in the odom frame in meters.
  double x_ = 0.0;

  /// @brief Estimated y position in the odom frame in meters.
  double y_ = 0.0;

  /// @brief Estimated yaw angle in the odom frame in radians.
  double theta_ = 0.0;

  /// @brief Real-time safe publisher for /odom.
  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry>> odom_pub_;

  /// @brief Velocity command frame: "chassis" or "global".
  std::string velocity_mode_;

  /// @brief World frame used in global velocity mode.
  std::string world_frame_ = "odom";

  /// @brief Listener used to transform the command in global velocity mode.
  std::unique_ptr<tf::TransformListener> tf_listener_;

  /// @brief Broadcaster publishing the odom -> base_link transform.
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace hero_chassis_controller

#endif
