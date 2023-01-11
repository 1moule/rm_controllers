#include "rm_shooter_controllers/pneumatic.h"
#include "pluginlib/class_list_macros.hpp"

namespace rm_shooter_controllers
{
bool PneumaticController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh,
                               ros::NodeHandle& controller_nh)
{
  Controller::init(robot_hw, root_nh, controller_nh);
  ros::NodeHandle nh_trigger = ros::NodeHandle(controller_nh, "trigger");
  ros::NodeHandle nh_putter = ros::NodeHandle(controller_nh, "putter");
  ros::NodeHandle nh_pump = ros::NodeHandle(controller_nh, "pump");

  return (ctrl_trigger_.init(effort_joint_interface_, nh_trigger) &&
          ctrl_putter_.init(effort_joint_interface_, nh_putter) && ctrl_pump_.init(effort_joint_interface_, nh_pump));
}

void PneumaticController::push(const ros::Time& time, const ros::Duration& period)
{
  if (state_changed_)
  {  // on enter
    state_changed_ = false;
    ROS_INFO("[Shooter] Enter PUSH");
  }
  if (((ros::Time::now() - last_shoot_time_).toSec() >= 1. / cmd_.hz) ||
      ((cmd_.speed == cmd_.SPEED_ZERO_FOR_TEST) && ((ros::Time::now() - last_shoot_time_).toSec() >= 1. / cmd_.hz)))
  {
    ctrl_trigger_.setCommand(ctrl_trigger_.command_struct_.position_ -
                             2. * M_PI / static_cast<double>(push_per_rotation_));
    if ((ros::Time::now() - last_trigger_time_).toSec() < trigger_threshold_)
      ctrl_putter_.setCommand(1);  // push bullet to pressure chamber
    // if (air pressure < qd_des (desire pressure) && (std::abs(ctrl_putter_.joint_.getPosition() -
    // ctrl_putter_.joint_.getCommand()) < putter_pos_threshold_;
    {
      ctrl_pump_.setCommand(500);  // start pumping
      if (!start_pump_flag_)
      {
        last_pump_time_ = ros::Time::now();
        start_pump_flag_ = true;
      }
    }
    // if(air pressure >= qd_des (desire pressure) || ros::Time::now() - last_pump_time_ > pump_threshold_)
    {
      ctrl_pump_.setCommand(0);    // stop
      ctrl_putter_.setCommand(0);  // cylinder recovery
      last_shoot_time_ = time;
      start_pump_flag_ = false;
    }
  }
  checkBlock(time);
}

void PneumaticController::stop(const ros::Time& time, const ros::Duration& period)
{
  if (state_changed_)
  {  // on enter
    state_changed_ = false;
    ROS_INFO("[Shooter] Enter STOP");

    ctrl_trigger_.setCommand(ctrl_trigger_.joint_.getPosition());
    ctrl_putter_.setCommand(0);
    ctrl_pump_.setCommand(0);
  }
}

void PneumaticController::normalize()
{
  double push_angle = 2. * M_PI / static_cast<double>(push_per_rotation_);
  ctrl_trigger_.setCommand(push_angle * std::floor((ctrl_trigger_.joint_.getPosition() + 0.01) / push_angle));
  ctrl_putter_.joint_.setCommand(0);
  ctrl_pump_.setCommand(0);
}

void PneumaticController::ctrlUpdate(const ros::Time& time, const ros::Duration& period)
{
  ctrl_trigger_.update(time, period);
  ctrl_putter_.update(time, period);
  ctrl_pump_.update(time, period);
}
}  // namespace rm_shooter_controllers
PLUGINLIB_EXPORT_CLASS(rm_shooter_controllers::PneumaticController, controller_interface::ControllerBase)
