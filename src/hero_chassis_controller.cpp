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

#include "hero_chassis_controller/hero_chassis_controller.h"

#include <pluginlib/class_list_macros.hpp>

namespace hero_chassis_controller
{
namespace
{
constexpr int LEFT_FRONT = 0;
constexpr int RIGHT_FRONT = 1;
constexpr int LEFT_BACK = 2;
constexpr int RIGHT_BACK = 3;

}  // namespace

bool HeroChassisController::init(hardware_interface::EffortJointInterface* effort_joint_interface,
                                 ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  // Read the velocity mode parameter
  if (!controller_nh.getParam("velocity_mode", velocity_mode_))
  {
    velocity_mode_ = "chassis";
  }
  controller_nh.param<std::string>("world_frame", world_frame_, "odom");

  // Initialize the tf listener
  tf_listener_.reset(new tf::TransformListener());
  tf_broadcaster_.reset(new tf2_ros::TransformBroadcaster());

  // Read the chassis geometry parameters
  if (!controller_nh.getParam("wheel_radius", wheel_radius_) ||
      !controller_nh.getParam("track_width_x", track_width_x_) ||
      !controller_nh.getParam("track_width_y", track_width_y_))
  {
    ROS_ERROR(
        "Failed to get chassis geometric parameters (wheel_radius, "
        "track_width_x, track_width_y)!");
    return false;
  }

  const std::string WHEEL_NAMES[4] = { "lf_wheel", "rf_wheel", "lb_wheel", "rb_wheel" };

  for (int i = 0; i < 4; ++i)
  {
    // Node handle of this wheel's parameters
    ros::NodeHandle wheel_nh(controller_nh, "wheels/" + wheel_names[i]);
    // Joint handle
    std::string joint_name;
    if (!wheel_nh.getParam("joint", joint_name))
    {
      ROS_ERROR_STREAM("No 'joint' parameter specified for " << wheel_names[i] << " in YAML");
      return false;
    }
    try
    {
      wheel_joints_[i] = effort_joint_interface->getHandle(joint_name);
    }
    catch (const hardware_interface::HardwareInterfaceException& e)
    {
      ROS_ERROR_STREAM("Failed to find a proper HW interface: " << e.what());
      return false;
    }

    // Initialize the PID controller
    ros::NodeHandle pid_nh(wheel_nh, "pid");
    if (!pid_controllers_[i].init(pid_nh))
    {
      ROS_ERROR_STREAM("Failed to initialize pid controllers for:" << wheel_names[i]);
      return false;
    }
  }

  // Subscriber for /cmd_vel
  sub_cmd_vel_ = root_nh.subscribe("/cmd_vel", 1, &HeroChassisController::cmdVelCallback, this);

  // Publisher for /odom
  odom_pub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(root_nh, "odom", 100));

  ROS_INFO("Hero Chassis Controller Instantiated");
  return true;
}

void HeroChassisController::starting(const ros::Time& time)
{
  // Clear the command buffer so that no stale command is applied on start
  geometry_msgs::Twist empty_twist;
  command_buffer_.writeFromNonRT(empty_twist);

  // Reset the odometry
  x_ = 0.0;
  y_ = 0.0;
  theta_ = 0.0;

  // Reset the historical error and integral term of the four PIDs
  for (int i = 0; i < 4; ++i)
  {
    pid_controllers_[i].reset();
  }
}

void HeroChassisController::update(const ros::Time& time, const ros::Duration& period)
{
  // Read the latest velocity command from /cmd_vel
  geometry_msgs::Twist* current_cmd = command_buffer_.readFromRT();
  double cmd_vx = current_cmd->linear.x;
  double cmd_vy = current_cmd->linear.y;
  double cmd_wz = current_cmd->angular.z;

  if (velocity_mode_ == "global")
  {
    try
    {
      // Build the velocity vector in the world frame
      tf::Vector3 vel_world_vector(cmd_vx, cmd_vy, 0.0);
      // Stamp the vector with the current time and the world frame
      tf::Stamped<tf::Vector3> stamped_vel_world(vel_world_vector, time, world_frame_);
      tf::Stamped<tf::Vector3> stamped_vel_body;

      // Transform the velocity vector into the chassis frame
      tf_listener_->transformVector("base_link", stamped_vel_world, stamped_vel_body);
      // Use the transformed velocity as the chassis frame command
      cmd_vx = stamped_vel_body.x();
      cmd_vy = stamped_vel_body.y();
    }
    catch (tf::TransformException& ex)
    {
      ROS_ERROR_THROTTLE(1.0, "%s", ex.what());
      cmd_vx = cmd_vy = cmd_wz = 0;
    }
  }
  // Sum of the half wheel base and half wheel track
  double L_sum = track_width_x_ / 2 + track_width_y_ / 2;
  double target_speeds[4];

  // Inverse kinematics: chassis velocity to wheel angular velocities
  target_speeds[LEFT_FRONT] = (cmd_vx - cmd_vy - L_sum * cmd_wz) / wheel_radius_;
  target_speeds[RIGHT_FRONT] = (cmd_vx + cmd_vy + L_sum * cmd_wz) / wheel_radius_;
  target_speeds[LEFT_BACK] = (cmd_vx + cmd_vy - L_sum * cmd_wz) / wheel_radius_;
  target_speeds[RIGHT_BACK] = (cmd_vx - cmd_vy + L_sum * cmd_wz) / wheel_radius_;

  for (int i = 0; i < 4; ++i)
  {
    // Velocity error = desired - measured wheel velocity
    double error = target_speeds[i] - wheel_joints_[i].getVelocity();

    // PID computes the output effort
    double commanded_effort = pid_controllers_[i].computeCommand(error, period);

    // Send the effort to the hardware interface
    wheel_joints_[i].setCommand(commanded_effort);
  }

  // Forward kinematics: wheel angular velocities to chassis velocity
  double w_lf = wheel_joints_[LEFT_FRONT].getVelocity();
  double w_rf = wheel_joints_[RIGHT_FRONT].getVelocity();
  double w_lb = wheel_joints_[LEFT_BACK].getVelocity();
  double w_rb = wheel_joints_[RIGHT_BACK].getVelocity();

  double vx_forward = wheel_radius_ * (w_lf + w_rf + w_lb + w_rb) / 4.0;
  double vy_forward = wheel_radius_ * (-w_lf + w_rf + w_lb - w_rb) / 4.0;
  double wz_forward = wheel_radius_ * (-w_lf + w_rf - w_lb + w_rb) / (4.0 * L_sum);

  double dt = period.toSec();
  double delta_x = (vx_forward * std::cos(theta_) - vy_forward * std::sin(theta_)) * dt;
  double delta_y = (vx_forward * std::sin(theta_) + vy_forward * std::cos(theta_)) * dt;
  double delta_theta = wz_forward * dt;

  x_ += delta_x;
  y_ += delta_y;
  theta_ += delta_theta;

  // Convert yaw to quaternion
  tf2::Quaternion q;
  q.setRPY(0, 0, theta_);

  if (odom_pub_ && odom_pub_->trylock())
  {
    // Fill the message header
    odom_pub_->msg_.header.stamp = time;
    odom_pub_->msg_.header.frame_id = "odom";
    odom_pub_->msg_.child_frame_id = "base_link";

    // Fill the pose
    odom_pub_->msg_.pose.pose.position.x = x_;
    odom_pub_->msg_.pose.pose.position.y = y_;
    odom_pub_->msg_.pose.pose.position.z = 0.0;
    odom_pub_->msg_.pose.pose.orientation = tf2::toMsg(q);

    // Fill the velocity
    odom_pub_->msg_.twist.twist.linear.x = vx_forward;
    odom_pub_->msg_.twist.twist.linear.y = vy_forward;
    odom_pub_->msg_.twist.twist.angular.z = wz_forward;

    // Publish
    odom_pub_->unlockAndPublish();
  }

  geometry_msgs::TransformStamped transform_stamped;
  transform_stamped.header.stamp = time;
  transform_stamped.header.frame_id = "odom";
  transform_stamped.child_frame_id = "base_link";
  transform_stamped.transform.translation.x = x_;
  transform_stamped.transform.translation.y = y_;
  transform_stamped.transform.translation.z = 0.0;
  transform_stamped.transform.rotation = tf2::toMsg(q);

  tf_broadcaster_.sendTransform(transform_stamped);
}

void HeroChassisController::stopping(const ros::Time& time)
{
  for (int i = 0; i < 4; ++i)
  {
    wheel_joints_[i].setCommand(0.0);
  }
}

void HeroChassisController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  command_buffer_.writeFromNonRT(*msg);
}

}  // namespace hero_chassis_controller

PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController, controller_interface::ControllerBase)
