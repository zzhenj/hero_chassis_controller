//
// Created by zane on 2026/11/6.
//

#ifndef HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H
#define HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H

#include <memory>
#include <string>

#include <controller_interface/controller.h>
#include <geometry_msgs/Twist.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_buffer.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <control_toolbox/pid.h>
#include <ros/time.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_listener.h>


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
   ros::Subscriber sub_cmd_vel_;
   realtime_tools::RealtimeBuffer<geometry_msgs::Twist> command_buffer_;

   void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);

   hardware_interface::JointHandle wheel_joints_[4];

   double WHEEL_RADIUS;
   double TRACK_WIDTH_X_;
   double TRACK_WIDTH_Y_;

   control_toolbox::Pid pid_controllers_[4];

   double x_ = 0.0;
   double y_ = 0.0;
   double theta_ = 0.0;

   std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry>> odom_pub_;

   std::string velocity_mode_;
   std::string world_frame_;

   std::unique_ptr<tf::TransformListener> tf_listener_;
   std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace hero_chassis_controller

#endif
