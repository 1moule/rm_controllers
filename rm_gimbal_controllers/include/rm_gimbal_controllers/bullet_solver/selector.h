//
// Created by guanlin on 25-4-3.
//

#pragma once

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace rm_gimbal_controllers
{
enum Armor
{
  LEFT = 0,
  FRONT,
  RIGHT,
  BACK,
};
enum SwitchArmorState
{
  NO_SWITCH = 0,
  READY_SWITCH,
  START_SWITCH,
};
class TargetSelector
{
public:
  TargetSelector() = default;
  void setTargetState(geometry_msgs::Point pos, geometry_msgs::Vector3 vel, double yaw, double v_yaw, double r1,
                      double r2, int armors_num)
  {
    pos_ = pos;
    vel_ = vel;
    yaw_ = yaw;
    v_yaw_ = v_yaw;
    r1_ = r1;
    r2_ = r2;
    armors_num_ = armors_num;
  }
  void configure(double delay, double bullet_speed, double resistance_coff, double gimbal_switch_duration,
                 double min_switch_angle, int min_switch_count, bool track_target)
  {
    delay_ = delay;
    bullet_speed_ = bullet_speed;
    gimbal_switch_duration_ = gimbal_switch_duration;
    resistance_coff_ = resistance_coff;
    min_switch_angle_ = min_switch_angle;
    min_switch_count_ = min_switch_count;
    track_target_ = track_target;
  }
  int getTargetArmor()
  {
    double target_rho = std::sqrt(std::pow(pos_.x, 2) + std::pow(pos_.y, 2));
    double output_yaw = std::atan2(pos_.y, pos_.x);
    double output_pitch = std::atan2(pos_.z, std::sqrt(std::pow(pos_.x, 2) + std::pow(pos_.y, 2)));
    double rough_fly_time =
        (-std::log(1 - target_rho * resistance_coff_ / (bullet_speed_ * std::cos(output_pitch)))) / resistance_coff_;
    double min_switch_angle = min_switch_angle_ / 180 * M_PI;
    switch_armor_angle_ = track_target_ ?
                              M_PI / armors_num_ - (2 * rough_fly_time + gimbal_switch_duration_) / 2 * abs(v_yaw_) :
                              min_switch_angle;
    if (track_target_)
    {
      if ((std::remainder(yaw_ - output_yaw, 2 * M_PI) > switch_armor_angle_ && v_yaw_ > 1.) ||
          (std::remainder(yaw_ - output_yaw, 2 * M_PI) < -switch_armor_angle_ && v_yaw_ < -1.))
      {
        switch_count_++;
        if (switch_count_ > min_switch_count_)
        {
          if ((std::remainder(yaw_ - (M_PI * 2 / armors_num_) - output_yaw, 2 * M_PI) > switch_armor_angle_ &&
               v_yaw_ > 1.) ||
              (std::remainder(yaw_ + (M_PI * 2 / armors_num_) - output_yaw, 2 * M_PI) < -switch_armor_angle_ &&
               v_yaw_ < -1.))
          {
            next_switch_count_++;
            if (next_switch_count_ > min_switch_count_)
              target_armor_ = BACK;
          }
          else
            target_armor_ = v_yaw_ > 0. ? LEFT : RIGHT;
        }
      }
      else
        target_armor_ = FRONT;

      // Judge ready switch
      int offset = (target_armor_ == BACK && v_yaw_ > 0.) ? -4 : 0;
      if (((std::remainder(yaw_ + (M_PI * 2 / armors_num_) * (target_armor_ + offset - 1) +
                               v_yaw_ * (delay_ - 0.001 * min_switch_count_) - output_yaw,
                           2 * M_PI)) > switch_armor_angle_ &&
           v_yaw_ > 1.) ||
          (std::remainder(yaw_ + (M_PI * 2 / armors_num_) * (target_armor_ - 1) +
                              v_yaw_ * (delay_ - 0.001 * min_switch_count_) - output_yaw,
                          2 * M_PI) < -switch_armor_angle_ &&
           v_yaw_ < -1.))
        switch_armor_state_ = READY_SWITCH;
      else
      {
        if (switch_armor_state_ == READY_SWITCH && (current_armor_ == LEFT || current_armor_ == RIGHT) &&
            target_armor_ == FRONT)
          switch_armor_state_ = READY_SWITCH;
        else
          switch_armor_state_ = NO_SWITCH;
      }
    }
    else
    {
      if ((std::remainder(yaw_ + v_yaw_ * (rough_fly_time + delay_) - output_yaw, 2 * M_PI) > switch_armor_angle_ &&
           v_yaw_ > 1.) ||
          (std::remainder(yaw_ + v_yaw_ * (rough_fly_time + delay_) - output_yaw, 2 * M_PI) < -switch_armor_angle_ &&
           v_yaw_ < -1.))
      {
        if ((std::remainder(yaw_ - (M_PI * 2 / armors_num_) + v_yaw_ * (rough_fly_time + delay_) - output_yaw,
                            2 * M_PI) > switch_armor_angle_ &&
             v_yaw_ > 1.) ||
            (std::remainder(yaw_ + (M_PI * 2 / armors_num_) + v_yaw_ * (rough_fly_time + delay_) - output_yaw,
                            2 * M_PI) < -switch_armor_angle_ &&
             v_yaw_ < -1.))
          target_armor_ = BACK;
        else
          target_armor_ = v_yaw_ > 0. ? LEFT : RIGHT;
      }
      else
        target_armor_ = FRONT;
    }

    if ((current_armor_ == FRONT && target_armor_ != FRONT) ||
        ((current_armor_ == LEFT || current_armor_ == RIGHT) && target_armor_ == BACK))
      switch_armor_state_ = START_SWITCH;
    if (((current_armor_ != FRONT) && target_armor_ == FRONT) ||
        (current_armor_ == BACK && (target_armor_ == LEFT || target_armor_ == RIGHT)))
    {
      switch_count_ = 0;
      next_switch_count_ = 0;
    }
    current_armor_ = target_armor_;

    return target_armor_;
  }
  int getSwitchArmorState()
  {
    return switch_armor_state_;
  }
  double getSwitchArmorAngle()
  {
    return switch_armor_angle_;
  }

private:
  geometry_msgs::Point pos_;
  geometry_msgs::Vector3 vel_;
  double yaw_{}, v_yaw_{}, r1_{}, r2_{};
  double delay_{}, bullet_speed_{}, gimbal_switch_duration_{}, resistance_coff_{}, min_switch_angle_{};
  double switch_armor_angle_{};
  int current_armor_{ 1 }, target_armor_{ 1 };
  int switch_armor_state_{ 0 };
  int switch_count_{ 0 }, next_switch_count_{ 0 };
  int armors_num_{ 4 };
  int min_switch_count_{ 3 };
  bool track_target_{ true };
};
}  // namespace rm_gimbal_controllers
