// created by guanlin on 2022/9/6

#pragma once

#include "rm_shooter_controllers/shooter_base.h"

namespace rm_shooter_controllers
{
class FrictionWheelController
  : public Controller<hardware_interface::EffortJointInterface, rm_control::RobotStateInterface>
{
public:
  FrictionWheelController() = default;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;

private:
  void ready(const ros::Time& time, const ros::Duration& period) override;
  void stop(const ros::Time& time, const ros::Duration& period) override;
  void push(const ros::Time& time, const ros::Duration& period) override;
  void ctrlUpdate(const ros::Time& time, const ros::Duration& period) override;
  void reconfigCB(rm_shooter_controllers::ShooterConfig& config, uint32_t /*level*/) override;
  void setspeed(const ros::Time& time, const ros::Duration& period);

  effort_controllers::JointVelocityController ctrl_friction_l_, ctrl_friction_r_;
};
}  // namespace rm_shooter_controllers
