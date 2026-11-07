#include <string>
#include <vector>

#include <ros/ros.h>

#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/SwitchController.h>

namespace
{

const char* const CONTROLLER_MANAGER_NAMESPACE = "/controller_manager";

bool loadAndStartControllers(ros::NodeHandle& node_handle,
                             const std::vector<std::string>& controllers)
{
  const std::string manager(CONTROLLER_MANAGER_NAMESPACE);

  ros::ServiceClient load_client = node_handle.serviceClient<controller_manager_msgs::LoadController>(
      manager + "/load_controller");
  ros::ServiceClient switch_client =
      node_handle.serviceClient<controller_manager_msgs::SwitchController>(manager +
                                                                          "/switch_controller");

  ROS_INFO("Waiting for %s to advertise its services", manager.c_str());
  if (!load_client.waitForExistence(ros::Duration(30.0)))
  {
    ROS_ERROR("controller_manager is not running, did you start the robot?");
    return false;
  }
  if (!switch_client.waitForExistence(ros::Duration(30.0)))
  {
    ROS_ERROR("switch_controller service is not available");
    return false;
  }

  for (const auto& controller : controllers)
  {
    controller_manager_msgs::LoadController load_service;
    load_service.request.name = controller;
    if (load_client.call(load_service) && load_service.response.ok)
    {
      ROS_INFO("Loaded controller %s", controller.c_str());
    }
    else
    {
      ROS_WARN("Could not load controller %s, assuming it is already loaded",
               controller.c_str());
    }
  }

  controller_manager_msgs::SwitchController switch_service;
  switch_service.request.start_controllers = controllers;
  switch_service.request.strictness = controller_manager_msgs::SwitchController::Request::STRICT;
  if (!switch_client.call(switch_service) || !switch_service.response.ok)
  {
    ROS_ERROR("Failed to start the controllers");
    return false;
  }

  ROS_INFO("Started %zu controller(s)", controllers.size());
  return true;
}

}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hero_chassis_controller");
  ros::NodeHandle node_handle("~");

  std::vector<std::string> controllers;
  if (!node_handle.getParam("controllers", controllers))
  {
    controllers = {"hero_chassis_controller", "joint_state_controller"};
  }

  if (!loadAndStartControllers(node_handle, controllers))
  {
    return 1;
  }

  ros::spin();
  return 0;
}
