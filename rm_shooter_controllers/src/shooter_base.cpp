/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, Qiayuan Liao
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

//
// Created by huakang on 2021/1/18.
//

#include <rm_common/ros_utilities.h>
#include <string>
#include "rm_shooter_controllers/shooter_base.h"

namespace rm_shooter_controllers
{
template class Controller<hardware_interface::EffortJointInterface, rm_control::RobotStateInterface>;

template <typename... T>
bool Controller<T...>::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh,
                            ros::NodeHandle& controller_nh)
{
  config_ = { .block_effort = getParam(controller_nh, "block_effort", 0.),
              .block_speed = getParam(controller_nh, "block_speed", 0.),
              .block_duration = getParam(controller_nh, "block_duration", 0.),
              .block_overtime = getParam(controller_nh, "block_overtime", 0.),
              .anti_block_angle = getParam(controller_nh, "anti_block_angle", 0.),
              .anti_block_threshold = getParam(controller_nh, "anti_block_threshold", 0.),
              .qd_10 = getParam(controller_nh, "qd_10", 0.),
              .qd_15 = getParam(controller_nh, "qd_15", 0.),
              .qd_16 = getParam(controller_nh, "qd_16", 0.),
              .qd_18 = getParam(controller_nh, "qd_18", 0.),
              .qd_30 = getParam(controller_nh, "qd_30", 0.),
              .lf_extra_rotat_speed = getParam(controller_nh, "lf_extra_rotat_speed", 0.) };
  config_rt_buffer.initRT(config_);
  push_per_rotation_ = getParam(controller_nh, "push_per_rotation", 0);
  push_qd_threshold_ = getParam(controller_nh, "push_qd_threshold", 0.);

  cmd_subscriber_ = controller_nh.subscribe<rm_msgs::ShootCmd>("command", 1, &Controller::commandCB, this);
  // Init dynamic reconfigure
  d_srv_ = new dynamic_reconfigure::Server<rm_shooter_controllers::ShooterConfig>(controller_nh);
  dynamic_reconfigure::Server<rm_shooter_controllers::ShooterConfig>::CallbackType cb = [this](auto&& PH1, auto&& PH2) {
    reconfigCB(std::forward<decltype(PH1)>(PH1), std::forward<decltype(PH2)>(PH2));
  };
  d_srv_->setCallback(cb);
  effort_joint_interface_ = robot_hw->get<hardware_interface::EffortJointInterface>();

  return true;
}

template <typename... T>
void Controller<T...>::starting(const ros::Time& /*time*/)
{
  state_ = STOP;
  state_changed_ = true;
}

template <typename... T>
void Controller<T...>::update(const ros::Time& time, const ros::Duration& period)
{
  cmd_ = *cmd_rt_buffer_.readFromRT();
  config_ = *config_rt_buffer.readFromRT();
  if (state_ != cmd_.mode && state_ != BLOCK)
  {
    state_ = cmd_.mode;
    state_changed_ = true;
  }
  setspeed(time, period);
  switch (state_)
  {
    case READY:
      ready(period);
      break;
    case PUSH:
      push(time, period);
      break;
    case STOP:
      stop(time, period);
      break;
    case BLOCK:
      block(time, period);
      break;
  }
  ctrlUpdate(time, period);
}

template <typename... T>
void Controller<T...>::ready(const ros::Duration& period)
{
  if (state_changed_)
  {  // on enter
    state_changed_ = false;
    ROS_INFO("[Shooter] Enter READY");

    normalize();
  }
}

template <typename... T>
void Controller<T...>::block(const ros::Time& time, const ros::Duration& period)
{
  if (state_changed_)
  {  // on enter
    state_changed_ = false;
    ROS_INFO("[Shooter] Enter BLOCK");
    last_block_time_ = time;
    ctrl_trigger_.setCommand(ctrl_trigger_.joint_.getPosition() + config_.anti_block_angle);
  }
  if (std::abs(ctrl_trigger_.command_struct_.position_ - ctrl_trigger_.joint_.getPosition()) <
          config_.anti_block_threshold ||
      (time - last_block_time_).toSec() > config_.block_overtime)
  {
    normalize();

    state_ = PUSH;
    state_changed_ = true;
    ROS_INFO("[Shooter] Exit BLOCK");
  }
}

template <typename... T>
void Controller<T...>::checkBlock(const ros::Time& time)
{
  // Check block
  if (ctrl_trigger_.joint_.getEffort() < -config_.block_effort &&
      std::abs(ctrl_trigger_.joint_.getVelocity()) < config_.block_speed)
  {
    if (!maybe_block_)
    {
      block_time_ = time;
      maybe_block_ = true;
    }
    if ((time - block_time_).toSec() >= config_.block_duration)
    {
      state_ = BLOCK;
      state_changed_ = true;
      ROS_INFO("[Shooter] Exit PUSH");
    }
  }
  else
    maybe_block_ = false;
}

template <typename... T>
void Controller<T...>::normalize()
{
  double push_angle = 2. * M_PI / static_cast<double>(push_per_rotation_);
  ctrl_trigger_.setCommand(push_angle * std::floor((ctrl_trigger_.joint_.getPosition() + 0.01) / push_angle));
}

template <typename... T>
void Controller<T...>::reconfigCB(rm_shooter_controllers::ShooterConfig& config, uint32_t /*level*/)
{
  ROS_INFO("[Shooter] Dynamic params change");
  if (!dynamic_reconfig_initialized_)
  {
    Config init_config = *config_rt_buffer.readFromNonRT();  // config init use yaml
    config.block_effort = init_config.block_effort;
    config.block_speed = init_config.block_speed;
    config.block_duration = init_config.block_duration;
    config.block_overtime = init_config.block_overtime;
    config.anti_block_angle = init_config.anti_block_angle;
    config.anti_block_threshold = init_config.anti_block_threshold;
    config.lf_extra_rotat_speed = init_config.lf_extra_rotat_speed;
    dynamic_reconfig_initialized_ = true;
  }
  Config config_non_rt{ .block_effort = config.block_effort,
                        .block_speed = config.block_speed,
                        .block_duration = config.block_duration,
                        .block_overtime = config.block_overtime,
                        .anti_block_angle = config.anti_block_angle,
                        .anti_block_threshold = config.anti_block_threshold };
  config_rt_buffer.writeFromNonRT(config_non_rt);
}
}  // namespace rm_shooter_controllers
