//
// Created by zane on 2026/11/6.
//

#ifndef HERO_CHASSIS_CONTROLLER_PID_H
#define HERO_CHASSIS_CONTROLLER_PID_H

#include <string>

#include <control_toolbox/pid.h>
#include <ros/duration.h>

namespace hero_chassis_controller
{

class PidController
{
 public:
  PidController() = default;
  ~PidController() = default;

  bool init(const std::string& joint_name, const std::string& parameter_namespace,
            double max_effort);

  double computeEffort(double target_velocity, double current_velocity,
                       const ros::Duration& period);

  void reset();

  double maxEffort() const;

 private:
  control_toolbox::Pid pid_;
  std::string joint_name_;
  double max_effort_{0.0};
};

}  // namespace hero_chassis_controller

#endif
