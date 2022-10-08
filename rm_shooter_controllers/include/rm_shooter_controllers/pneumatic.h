// created by guanlin on 2022/9/6

#pragma once

#include "rm_shooter_controllers/shooter_base.h"
#include "gpio_controller/gpio_controller.h"

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
  void reachSpeed(double qd_des) override;
  void normalize() override;
  void ctrlUpdate(const ros::Time& time, const ros::Duration& period) override;

  effort_controllers::JointPositionController ctrl_putter_;
  effort_controllers::JointVelocityController ctrl_pump_;
  gpio_controller::Controller ctrl_valve;
  ros::Publisher gpio_cmd_publisher_;
  rm_msgs::GpioData msg_{};
};
}  // namespace rm_shooter_controllers
