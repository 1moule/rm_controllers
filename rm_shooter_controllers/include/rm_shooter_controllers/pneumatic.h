// created by guanlin on 2022/9/6

#pragma once

#include "rm_shooter_controllers/shooter_base.h"

namespace rm_shooter_controllers
{
class PneumaticController : public Controller<hardware_interface::EffortJointInterface, rm_control::RobotStateInterface>
{
public:
  PneumaticController() = default;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;

private:
  void stop(const ros::Time& time, const ros::Duration& period) override;
  void push(const ros::Time& time, const ros::Duration& period) override;
  void setspeed(const ros::Time& time, const ros::Duration& period) override;
  void ctrlUpdate(const ros::Time& time, const ros::Duration& period) override;
  void reconfigCB(rm_shooter_controllers::ShooterConfig& config, uint32_t /*level*/) override;
};
}  // namespace rm_shooter_controllers
