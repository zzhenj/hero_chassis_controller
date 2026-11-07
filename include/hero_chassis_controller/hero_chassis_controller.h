//
// Created by zane on 2026/11/6.
//

#ifndef HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H
#define HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H

#include <array>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <controller_interface/controller.h>
#include <geometry_msgs/Twist.h>
#include <hardware_interface/joint_command_interface.h>
#include <nav_msgs/Odometry.h>
#include <realtime_tools/realtime_buffer.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <ros/time.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include "hero_chassis_controller/pid.h"

namespace hero_chassis_controller
{

class HeroChassisController
    : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
 public:
  HeroChassisController() = default;
  ~HeroChassisController() override = default;

  bool init(hardware_interface::EffortJointInterface* effort_joint_interface,
            ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;

  void starting(const ros::Time& time) override;

  void update(const ros::Time& time, const ros::Duration& period) override;

  void stopping(const ros::Time& time) override;

 private:
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);

  void computeWheelVelocity(const Eigen::Vector3d& chassis_velocity,
                            Eigen::Vector4d* wheel_velocity) const;

  Eigen::Vector3d computeChassisVelocity(const Eigen::Vector4d& wheel_velocity) const;

  bool transformToChassis(const ros::Time& time, Eigen::Vector3d* velocity);

  void publishOdometry(const ros::Time& time, const ros::Duration& period,
                       const Eigen::Vector3d& chassis_velocity);

  std::array<std::string, 4> joint_names_;
  std::array<hardware_interface::JointHandle, 4> joint_handles_;

  std::array<PidController, 4> pid_;
  double max_effort_{5.0};

  static constexpr double WHEEL_RADIUS = 0.0675;
  static constexpr double WHEEL_TRACK = 0.4;
  static constexpr double WHEEL_BASE = 0.4;

  realtime_tools::RealtimeBuffer<geometry_msgs::Twist> cmd_velocity_buffer_;
  ros::Subscriber cmd_velocity_sub_;

  ros::Publisher odom_pub_;
  tf::TransformBroadcaster tf_broadcaster_;
  tf::TransformListener tf_listener_;

  double odom_x_{0.0};
  double odom_y_{0.0};
  double odom_yaw_{0.0};
  bool odom_initialized_{false};

  bool use_world_velocity_{false};
  std::string world_frame_{"odom"};
  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_link"};

  double max_linear_acceleration_{0.0};
  Eigen::Vector3d last_cmd_velocity_{Eigen::Vector3d::Zero()};
  bool has_last_cmd_{false};
};

}  // namespace hero_chassis_controller

#endif
