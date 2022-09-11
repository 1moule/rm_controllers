// created by guanlin on 2022/9/6

#include "rm_shooter_controllers/pneumatic.h"
#include "pluginlib/class_list_macros.hpp"

namespace rm_shooter_controllers
{
bool PneumaticController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh,
                               ros::NodeHandle& controller_nh)
{
  Controller::init(robot_hw, root_nh, controller_nh);

  return true;
}

void PneumaticController::push(const ros::Time& time, const ros::Duration& period)
{
}

void PneumaticController::stop(const ros::Time& time, const ros::Duration& period)
{
}

void PneumaticController::setspeed(const ros::Time& time, const ros::Duration& period)
{
}

void PneumaticController::ctrlUpdate(const ros::Time& time, const ros::Duration& period)
{
}

void PneumaticController::reconfigCB(rm_shooter_controllers::ShooterConfig& config, uint32_t /*level*/)
{
}
}  // namespace rm_shooter_controllers
PLUGINLIB_EXPORT_CLASS(rm_shooter_controllers::PneumaticController, controller_interface::ControllerBase)
