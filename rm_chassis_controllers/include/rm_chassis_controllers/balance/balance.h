//
// Created by yezi on 2022/11/15.
//

#pragma once

#include "rm_common/lqr.h"
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <rm_msgs/BalanceState.h>
#include "rm_common/filters/kalman_filter.h"

#include "../chassis_base.h"

namespace rm_chassis_controllers
{
using Eigen::Matrix;
class BalanceController : public ChassisBase<rm_control::RobotStateInterface, hardware_interface::ImuSensorInterface,
                                             hardware_interface::EffortJointInterface>
{
  enum BalanceMode
  {
    NORMAL,
  };

public:
  BalanceController() = default;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;

private:
  void updateEstimation(const ros::Time& time, const ros::Duration& period);
  void moveJoint(const ros::Time& time, const ros::Duration& period) override;
  void normal(const ros::Time& time, const ros::Duration& period);
  geometry_msgs::Twist odometry() override;
  static const int STATE_DIM = 6;
  static const int CONTROL_DIM = 2;
  Eigen::Matrix<double, CONTROL_DIM, STATE_DIM> k_{};
  Eigen::Matrix<double, STATE_DIM, STATE_DIM> a_{}, q_{};
  Eigen::Matrix<double, STATE_DIM, CONTROL_DIM> b_{};
  Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM> r_{};
  Eigen::Matrix<double, STATE_DIM, 1> x_left_, x_right_;
  double vmc_bias_angle_, left_angle[2], right_angle[2], left_pos_[2], left_spd_[2], right_pos_[2], right_spd_[2];
  double wheel_radius_ = 0.06, wheel_track_ = 0.49;
  double body_mass_ = 10.717, g_ = 9.81;
  double position_des_ = 0;
  double position_offset_ = 0.;
  double position_clear_threshold_ = 0.;
  double yaw_des_ = 0;

  int balance_mode_;

  hardware_interface::ImuSensorHandle imu_handle_;
  hardware_interface::JointHandle left_wheel_joint_handle_, right_wheel_joint_handle_, left_front_leg_joint_handle_,
      left_back_leg_joint_handle_, right_front_leg_joint_handle_, right_back_leg_joint_handle_;

  control_toolbox::Pid pid_yaw_vel_, pid_left_leg_, pid_right_leg_, pid_theta_diff_, pid_length_diff_, pid_roll_,
      pid_center_gravity_;

  typedef std::shared_ptr<realtime_tools::RealtimePublisher<rm_msgs::BalanceState>> RtpublisherPtr;
  RtpublisherPtr state_pub_;
  geometry_msgs::Vector3 angular_vel_base_;
  double roll_, pitch_, yaw_;
  double leg_length_;
};

}  // namespace rm_chassis_controllers
