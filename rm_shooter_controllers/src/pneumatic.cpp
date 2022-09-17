// created by guanlin on 2022/9/6

#include "rm_shooter_controllers/pneumatic.h"
#include "pluginlib/class_list_macros.hpp"

namespace rm_shooter_controllers
{
bool PneumaticController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh,
                               ros::NodeHandle& controller_nh)
{
  Controller::init(robot_hw, root_nh, controller_nh);
  ros::NodeHandle root_nh_valve = ros::NodeHandle(root_nh, "valve");
  ros::NodeHandle controller_nh_valve = ros::NodeHandle(controller_nh, "valve");
  ros::NodeHandle nh_trigger = ros::NodeHandle(controller_nh, "trigger");
  cmd_publisher_ = controller_nh.advertise<rm_msgs::GpioData>("gpio_controller/command", 1);
  msg_.gpio_name[0] = "putter", msg_.gpio_name[1] = "vent";
  return (ctrl_trigger_.init(effort_joint_interface_, nh_trigger) &&
          ctrl_valve.init(robot_hw, root_nh_valve, controller_nh_valve));
}

void PneumaticController::push(const ros::Time& time, const ros::Duration& period)
{
  if (state_changed_)
  {  // on enter
    state_changed_ = false;
    ROS_INFO("[Shooter] Enter PUSH");
  }
  // if(no bullet in bomb zone && air pressure < threshold && （(ros::Time::now() - last_shoot_time_).toSec() >= 1. / cmd_.hz)
  // || (cmd_.speed == cmd_.SPEED_ZERO_FOR_TEST && (ros::Time::now() - last_shoot_time_).toSec() >= 1. / cmd_.hz))
  {
    ctrl_trigger_.setCommand(ctrl_trigger_.command_struct_.position_ -
                             2. * M_PI / static_cast<double>(push_per_rotation_));
    msg_.gpio_state[0] = rm_control::OUTPUT;
    cmd_publisher_.publish(msg_);  // push bullet to pressure chamber
    // if(no bullet in bomb zone && air pressure > threshold)  //bullet in bomb zone
    {
      msg_.gpio_state[1] = rm_control::OUTPUT;  // shoot
      cmd_publisher_.publish(msg_);
    }
    msg_.gpio_state[0] = rm_control::INPUT;
    cmd_publisher_.publish(msg_);  // cylinder recovery
    last_shoot_time_ = time;
  }
}

void PneumaticController::stop(const ros::Time& time, const ros::Duration& period)
{
  if (state_changed_)
  {  // on enter
    state_changed_ = false;
    ROS_INFO("[Shooter] Enter STOP");

    ctrl_trigger_.setCommand(ctrl_trigger_.joint_.getPosition());
    msg_.gpio_state[0] = rm_control::INPUT;
    msg_.gpio_state[1] = rm_control::INPUT;
    cmd_publisher_.publish(msg_);
  }
}

void PneumaticController::reachSpeed(double qd_des)
{
  // TODO: According to qd_des, change the pressure of the pressure chamber
}

void PneumaticController::ctrlUpdate(const ros::Time& time, const ros::Duration& period)
{
  ctrl_valve.update(time, period);
  ctrl_trigger_.update(time, period);
}
}  // namespace rm_shooter_controllers
PLUGINLIB_EXPORT_CLASS(rm_shooter_controllers::PneumaticController, controller_interface::ControllerBase)
