#include "hero_chassis_controller/pid.h"

#include <algorithm>

namespace hero_chassis_controller
{

bool PidController::init(const std::string& joint_name, const std::string& parameter_namespace,
                         double max_effort)
{
  joint_name_ = joint_name;
  max_effort_ = max_effort;

  const std::string prefix = parameter_namespace + "/pid/" + joint_name_;
  if (!pid_.initParam(prefix))
  {
    return false;
  }

  return true;
}

double PidController::computeEffort(double target_velocity, double current_velocity,
                                    const ros::Duration& period)
{
  const double error = target_velocity - current_velocity;
  double effort = pid_.computeCommand(error, period);

  effort = std::max(-max_effort_, std::min(max_effort_, effort));

  return effort;
}

void PidController::reset()
{
  pid_.reset();
}

double PidController::maxEffort() const
{
  return max_effort_;
}

}  // namespace hero_chassis_controller
