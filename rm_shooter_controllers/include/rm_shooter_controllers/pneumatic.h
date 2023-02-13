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
  void normalize() override;
  void ctrlUpdate(const ros::Time& time, const ros::Duration& period) override;

  effort_controllers::JointPositionController ctrl_putter_;
  effort_controllers::JointVelocityController ctrl_pump_;

  ros::Time last_trigger_time_, last_pump_time_;
  ros::Duration pump_duration_;

  double trigger_threshold_{}, putter_pos_threshold_{};
  bool start_pump_flag_ = false;
};
}  // namespace rm_shooter_controllers
