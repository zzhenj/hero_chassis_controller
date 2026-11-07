#include "hero_chassis_controller/hero_chassis_controller.h"

#include <algorithm>
#include <cmath>

#include <pluginlib/class_list_macros.hpp>

namespace hero_chassis_controller
{

namespace
{

const std::vector<std::string> DEFAULT_JOINT_NAMES = {
    "left_front_wheel_joint", "right_front_wheel_joint",
    "left_back_wheel_joint", "right_back_wheel_joint"};

}  // namespace

bool HeroChassisController::init(
    hardware_interface::EffortJointInterface* effort_joint_interface,
    ros::NodeHandle& /*root_nh*/, ros::NodeHandle& controller_nh)
{
  controller_nh.param("max_effort", max_effort_, 5.0);
  controller_nh.param("use_world_velocity", use_world_velocity_, false);
  controller_nh.param<std::string>("world_frame", world_frame_, "odom");
  controller_nh.param<std::string>("odom_frame", odom_frame_, "odom");
  controller_nh.param<std::string>("base_frame", base_frame_, "base_link");
  controller_nh.param("max_linear_acceleration", max_linear_acceleration_, 0.0);

  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    if (!controller_nh.getParam("joint_names/" + std::to_string(i), joint_names_[i]))
    {
      joint_names_[i] = DEFAULT_JOINT_NAMES[i];
    }
    joint_handles_[i] = effort_joint_interface->getHandle(joint_names_[i]);

    if (!pid_[i].init(joint_names_[i], controller_nh.getNamespace(), max_effort_))
    {
      ROS_ERROR_STREAM("hero_chassis_controller: missing PID gains for " << joint_names_[i]);
      return false;
    }
  }

  cmd_velocity_buffer_.writeFromNonRT(geometry_msgs::Twist());
  cmd_velocity_sub_ = controller_nh.subscribe<geometry_msgs::Twist>(
      "cmd_vel", 1, &HeroChassisController::cmdVelCallback, this);

  odom_pub_ = controller_nh.advertise<nav_msgs::Odometry>("odom", 10);

  return true;
}

void HeroChassisController::starting(const ros::Time& /*time*/)
{
  for (auto& wheel_pid : pid_)
  {
    wheel_pid.reset();
  }
  odom_x_ = 0.0;
  odom_y_ = 0.0;
  odom_yaw_ = 0.0;
  odom_initialized_ = false;
  has_last_cmd_ = false;
  last_cmd_velocity_.setZero();
  cmd_velocity_buffer_.writeFromNonRT(geometry_msgs::Twist());
}

void HeroChassisController::update(const ros::Time& time, const ros::Duration& period)
{
  (void)time;
  (void)period;

  for (auto& joint_handle : joint_handles_)
  {
    joint_handle.setCommand(0.0);
  }
}

void HeroChassisController::stopping(const ros::Time& /*time*/)
{
  for (auto& joint_handle : joint_handles_)
  {
    joint_handle.setCommand(0.0);
  }
}

void HeroChassisController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  cmd_velocity_buffer_.writeFromNonRT(*msg);
}

void HeroChassisController::computeWheelVelocity(const Eigen::Vector3d& chassis_velocity,
                                                 Eigen::Vector4d* wheel_velocity) const
{
  (void)chassis_velocity;
  *wheel_velocity = Eigen::Vector4d::Zero();
}

Eigen::Vector3d HeroChassisController::computeChassisVelocity(
    const Eigen::Vector4d& wheel_velocity) const
{
  (void)wheel_velocity;
  return Eigen::Vector3d::Zero();
}

bool HeroChassisController::transformToChassis(const ros::Time& time, Eigen::Vector3d* velocity)
{
  geometry_msgs::Vector3Stamped velocity_in;
  velocity_in.header.stamp = time;
  velocity_in.header.frame_id = world_frame_;
  velocity_in.vector.x = velocity->x();
  velocity_in.vector.y = velocity->y();
  velocity_in.vector.z = 0.0;

  geometry_msgs::Vector3Stamped velocity_out;
  try
  {
    tf_listener_.transformVector(base_frame_, velocity_in, velocity_out);
  }
  catch (const tf::TransformException& exception)
  {
    ROS_WARN_THROTTLE(1.0, "hero_chassis_controller: tf lookup failed: %s", exception.what());
    return false;
  }

  velocity->x() = velocity_out.vector.x;
  velocity->y() = velocity_out.vector.y;
  return true;
}

void HeroChassisController::publishOdometry(const ros::Time& time, const ros::Duration& period,
                                            const Eigen::Vector3d& chassis_velocity)
{
  (void)time;
  (void)period;
  (void)chassis_velocity;
}

}  // namespace hero_chassis_controller

PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController,
                       controller_interface::ControllerBase)
