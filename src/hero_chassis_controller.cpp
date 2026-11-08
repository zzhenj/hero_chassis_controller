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
bool HeroChassisController::init(
    hardware_interface::EffortJointInterface* effort_joint_interface,
    ros::NodeHandle& root_nh,ros::NodeHandle& controller_nh)
{
  //读取速度模式参数
  if (!controller_nh.getParam("velocity_mode",velocity_mode_))
  {
    velocity_mode_ = "chassis";
  }
  controller_nh.param<std::string>("world_frame", world_frame_, "odom");

  //初始化tf监听器
  tf_listener_.reset(new tf::TransformListener());
  tf_broadcaster_.reset(new tf2_ros::TransformBroadcaster());

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
  // /cmd_vel订阅器
  sub_cmd_vel_= root_nh.subscribe("/cmd_vel", 1, &HeroChassisController::cmdVelCallback, this);

  //初始化/odom
  odom_pub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(root_nh, "odom", 100));

  ROS_INFO("Hero Chassis Controller Instantiated");
  return true;
}

void HeroChassisController::starting(const ros::Time& time)
{
  //启动时清空速度指令缓冲区，防止带着历史旧指令突变
  geometry_msgs::Twist empty_twist;
  command_buffer_.writeFromNonRT(empty_twist);

  //重置odom
  x_ = 0.0;
  y_ = 0.0;
  theta_ = 0.0;

  // 重置 4 个 PID 的历史误差和积分项
  for (int i = 0; i < 4; ++i)
  {
    pid_controllers_[i].reset();
  }
}

void HeroChassisController::update(const ros::Time& time, const ros::Duration& period)
{
  //获取上层 /cmd_vel 的最新速度指令
  geometry_msgs::Twist* current_cmd = command_buffer_.readFromRT();
  double cmd_vx = current_cmd->linear.x;
  double cmd_vy = current_cmd->linear.y;
  double cmd_wz = current_cmd->angular.z;

  if (velocity_mode_ == "global")
  {
    try
    {
      //构造世界坐标系下的速度矢量
      tf::Vector3 vel_world_vector(cmd_vx,cmd_vy,0.0);
      //带有时间戳和源坐标系的Stamped矢量
      tf::Stamped<tf::Vector3> stamped_vel_world(vel_world_vector, time, world_frame_);
      tf::Stamped<tf::Vector3> stamped_vel_body;

      //调用transformVector转换到底盘坐标系 "base_link"
      tf_listener_->transformVector("base_link", stamped_vel_world, stamped_vel_body);
      //将变换后的结果赋值给底盘局部速度
      cmd_vx = stamped_vel_body.x();
      cmd_vy = stamped_vel_body.y();
    }
    catch (tf::TransformException& ex)
    {
      ROS_ERROR_THROTTLE(1.0, "%s", ex.what());
      cmd_vx = cmd_vy = cmd_wz = 0;
    }
  }
  //计算几何距离和
  double L_sum = TRACK_WIDTH_X_/2 + TRACK_WIDTH_Y_/2;
  double target_speeds[4];

  //应用麦克纳姆轮逆运动学公式计算四个轮子的期望转速
  target_speeds[LEFT_FRONT]  = (cmd_vx - cmd_vy - L_sum * cmd_wz) / WHEEL_RADIUS;
  target_speeds[RIGHT_FRONT] = (cmd_vx + cmd_vy + L_sum * cmd_wz) / WHEEL_RADIUS;
  target_speeds[LEFT_BACK]   = (cmd_vx + cmd_vy - L_sum * cmd_wz) / WHEEL_RADIUS;
  target_speeds[RIGHT_BACK]  = (cmd_vx - cmd_vy + L_sum * cmd_wz) / WHEEL_RADIUS;

  for (int i = 0; i < 4; ++i)
  {
    // 速度误差 = 期望转速 - 真实反馈转速
    double error = target_speeds[i] - wheel_joints_[i].getVelocity();

    // PID 计算输出力矩
    double commanded_effort = pid_controllers_[i].computeCommand(error,period);

    // 下发给硬件仿真接口
    wheel_joints_[i].setCommand(commanded_effort);
  }

  //正运动学解算
  double w_lf = wheel_joints_[LEFT_FRONT].getVelocity();
  double w_rf = wheel_joints_[RIGHT_FRONT].getVelocity();
  double w_lb = wheel_joints_[LEFT_BACK].getVelocity();
  double w_rb = wheel_joints_[RIGHT_BACK].getVelocity();

  double vx_forward = WHEEL_RADIUS*(w_lf + w_rf + w_lb + w_rb)/4.0;
  double vy_forward = WHEEL_RADIUS*(-w_lf + w_rf + w_lb - w_rb)/4.0;
  double wz_forward = WHEEL_RADIUS*(-w_lf + w_rf - w_lb + w_rb)/(4.0*L_sum);

  double dt = period.toSec();
  double delta_x = (vx_forward * std::cos(theta_) - vy_forward * std::sin(theta_)) * dt;
  double delta_y = (vx_forward * std::sin(theta_) + vy_forward * std::cos(theta_)) * dt;
  double delta_theta = wz_forward * dt;

  x_ += delta_x;
  y_ += delta_y;
  theta_ += delta_theta;

  //欧拉角转四元数
  tf2::Quaternion q;
  q.setRPY(0, 0, theta_);

  if (odom_pub_ && odom_pub_->trylock())
  {
    //填充消息头
    odom_pub_->msg_.header.stamp = time;
    odom_pub_->msg_.header.frame_id = "odom";
    odom_pub_->msg_.child_frame_id = "base_link";

    //填充位姿
    odom_pub_->msg_.pose.pose.position.x = x_;
    odom_pub_->msg_.pose.pose.position.y = y_;
    odom_pub_->msg_.pose.pose.position.z = 0.0;
    odom_pub_->msg_.pose.pose.orientation = tf2::toMsg(q);

    //填充速度
    odom_pub_->msg_.twist.twist.linear.x = vx_forward;
    odom_pub_->msg_.twist.twist.linear.y = vy_forward;
    odom_pub_->msg_.twist.twist.angular.z = wz_forward;

    //提交消息
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

};  // namespace hero_chassis_controller

PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController,
                       controller_interface::ControllerBase)
