#include "hero_chassis_controller/hero_chassis_controller.h"

#include <algorithm>
#include <cmath>

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
bool HeroChassisController::init(
    hardware_interface::EffortJointInterface* effort_joint_interface,
    ros::NodeHandle& root_nh,ros::NodeHandle& controller_nh)
{
  //读取底盘几何尺寸参数
  if (!controller_nh.getParam("wheel_radius", WHEEL_RADIUS) ||
      !controller_nh.getParam("track_width_x", TRACK_WIDTH_X_) ||
      !controller_nh.getParam("track_width_y", TRACK_WIDTH_Y_))
  {
    ROS_ERROR("Failed to get chassis geometric parameters (wheel_radius, track_width_x, track_width_y)!");
    return false;
  }

  const std::string wheel_names[4] = {"lf_wheel", "rf_wheel", "lb_wheel", "rb_wheel"};

  for (int i = 0; i < 4; ++i)
  {
    //四个轮子的句柄
    ros::NodeHandle wheel_nh(controller_nh, "wheels/" + wheel_names[i]);
    //四个关节句柄
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

    //初始化PID控制器
    ros::NodeHandle pid_nh(wheel_nh,"pid");
    if (!pid_controllers_[i].init(pid_nh))
    {
      ROS_ERROR_STREAM("Failed to initialize pid controllers for:" << wheel_names[i]);
      return false;
    }
  }

  sub_cmd_vel_= root_nh.subscribe("/cmd_vel", 1, &HeroChassisController::cmdVelCallback, this);

  ROS_INFO("Hero Chassis Controller Instantiated");
  return true;
}

void HeroChassisController::starting(const ros::Time& time)
{
  //启动时清空速度指令缓冲区，防止带着历史旧指令突变
  geometry_msgs::Twist empty_twist;
  command_buffer_.writeFromNonRT(empty_twist);

  // 重置 4 个 PID 的历史误差和积分项
  for (int i = 0; i < 4; ++i)
  {
    pid_controllers_[i].reset();
  }
}

void HeroChassisController::update(const ros::Time& time, const ros::Duration& period)
{
  //无锁获取上层 /cmd_vel 的最新速度指令
  geometry_msgs::Twist* current_cmd = command_buffer_.readFromRT();
  double vx = current_cmd->linear.x;
  double vy = current_cmd->linear.y;
  double wz = current_cmd->angular.z;

  //计算几何距离和
  double L_sum = TRACK_WIDTH_X_/2 + TRACK_WIDTH_Y_/2;
  double target_speeds[4];

  //应用麦克纳姆轮逆运动学公式计算四个轮子的期望转速
  target_speeds[LEFT_FRONT]  = (vx - vy - L_sum * wz) / WHEEL_RADIUS;
  target_speeds[RIGHT_FRONT] = (vx + vy + L_sum * wz) / WHEEL_RADIUS;
  target_speeds[LEFT_BACK]   = (vx + vy - L_sum * wz) / WHEEL_RADIUS;
  target_speeds[RIGHT_BACK]  = (vx - vy + L_sum * wz) / WHEEL_RADIUS;

  for (int i = 0; i < 4; ++i)
  {
    // 速度误差 = 期望转速 - 真实反馈转速
    double error = target_speeds[i] - wheel_joints_[i].getVelocity();

    // PID 计算输出力矩
    double commanded_effort = pid_controllers_[i].computeCommand(error,period);

    // 下发给硬件仿真接口
    wheel_joints_[i].setCommand(commanded_effort);
  }
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

};  // namespace hero_chassis_controller

PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController,
                       controller_interface::ControllerBase)
