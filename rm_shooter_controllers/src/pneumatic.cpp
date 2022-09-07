// created by guanlin on 2022/9/6

#include "rm_shooter_controllers/pneumatic.h"

namespace rm_shooter_controllers
{
bool PneumaticController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh,
                               ros::NodeHandle& controller_nh)
{
  Controller::init(robot_hw, root_nh, controller_nh);

  return true;
}
}  // namespace rm_shooter_controllers
