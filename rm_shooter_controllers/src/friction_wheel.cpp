//
// created by guanlin on 2022/9/6
//

#include "rm_shooter_controllers/friction_wheel.h"
namespace rm_shooter_controllers
{
bool FrictionWheelController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh,
                                   ros::NodeHandle& controller_nh)
{
  Controller::init(robot_hw, root_nh, controller_nh);
  ros::NodeHandle nh_friction_l = ros::NodeHandle(controller_nh, "friction_left");
  ros::NodeHandle nh_friction_r = ros::NodeHandle(controller_nh, "friction_right");
  ros::NodeHandle nh_trigger = ros::NodeHandle(controller_nh, "trigger");
  return (ctrl_friction_l_.init(effort_joint_interface_, nh_friction_l) &&
          ctrl_friction_r_.init(effort_joint_interface_, nh_friction_r) &&
          ctrl_trigger_.init(effort_joint_interface_, nh_trigger));
}

void FrictionWheelController::stop(const ros::Time& time, const ros::Duration& period)
{
  if (state_changed_)
  {  // on enter
    state_changed_ = false;
    ROS_INFO("[Shooter] Enter STOP");

    ctrl_friction_l_.setCommand(0.);
    ctrl_friction_r_.setCommand(0.);
    ctrl_trigger_.setCommand(ctrl_trigger_.joint_.getPosition());
  }
}

void FrictionWheelController::ready(const ros::Time& time, const ros::Duration& period)
{
  if (state_changed_)
  {  // on enter
    state_changed_ = false;
    ROS_INFO("[Shooter] Enter READY");

    setspeed(time, period);
    normalize();
  }
}

void FrictionWheelController::push(const ros::Time& time, const ros::Duration& period)
{
  if (state_changed_)
  {  // on enter
    state_changed_ = false;
    ROS_INFO("[Shooter] Enter PUSH");
  }
  setspeed(time, period);
  if ((cmd_.speed == cmd_.SPEED_ZERO_FOR_TEST && (ros::Time::now() - last_shoot_time_).toSec() >= 1. / cmd_.hz) ||
      (ctrl_friction_l_.joint_.getVelocity() >= push_qd_threshold_ * ctrl_friction_l_.command_ &&
       ctrl_friction_l_.joint_.getVelocity() > M_PI &&
       ctrl_friction_r_.joint_.getVelocity() <= push_qd_threshold_ * ctrl_friction_r_.command_ &&
       ctrl_friction_r_.joint_.getVelocity() < -M_PI && (ros::Time::now() - last_shoot_time_).toSec() >= 1. / cmd_.hz))
  {  // Time to shoot!!!
    ctrl_trigger_.setCommand(ctrl_trigger_.command_struct_.position_ -
                             2. * M_PI / static_cast<double>(push_per_rotation_));
    last_shoot_time_ = time;
  }
  else
    ROS_DEBUG("[Shooter] Wait for friction wheel");
  checkBlock(time, period);
}

void FrictionWheelController::setspeed(const ros::Time& time, const ros::Duration& period)
{
  double qd_des;
  if (cmd_.speed == cmd_.SPEED_10M_PER_SECOND)
    qd_des = config_.qd_10;
  else if (cmd_.speed == cmd_.SPEED_15M_PER_SECOND)
    qd_des = config_.qd_15;
  else if (cmd_.speed == cmd_.SPEED_16M_PER_SECOND)
    qd_des = config_.qd_16;
  else if (cmd_.speed == cmd_.SPEED_18M_PER_SECOND)
    qd_des = config_.qd_18;
  else if (cmd_.speed == cmd_.SPEED_30M_PER_SECOND)
    qd_des = config_.qd_30;
  else
    qd_des = 0.;
  ctrl_friction_l_.setCommand(qd_des + config_.lf_extra_rotat_speed);
  ctrl_friction_r_.setCommand(-qd_des);
}

void FrictionWheelController::ctrlUpdate(const ros::Time& time, const ros::Duration& period)
{
  ctrl_friction_l_.update(time, period);
  ctrl_friction_r_.update(time, period);
  ctrl_trigger_.update(time, period);
}

void FrictionWheelController::reconfigCB(rm_shooter_controllers::ShooterConfig& config, uint32_t /*level*/)
{
  ROS_INFO("[Shooter] Dynamic params change");
  if (!dynamic_reconfig_initialized_)
  {
    Config init_config = *config_rt_buffer.readFromNonRT();  // config init use yaml
    config.qd_10 = init_config.qd_10;
    config.qd_15 = init_config.qd_15;
    config.qd_16 = init_config.qd_16;
    config.qd_18 = init_config.qd_18;
    config.qd_30 = init_config.qd_30;
    config.lf_extra_rotat_speed = init_config.lf_extra_rotat_speed;
    dynamic_reconfig_initialized_ = true;
  }
  Config config_non_rt{ .qd_10 = config.qd_10,
                        .qd_15 = config.qd_15,
                        .qd_16 = config.qd_16,
                        .qd_18 = config.qd_18,
                        .qd_30 = config.qd_30,
                        .lf_extra_rotat_speed = config.lf_extra_rotat_speed };
  config_rt_buffer.writeFromNonRT(config_non_rt);
}
}  // namespace rm_shooter_controllers
