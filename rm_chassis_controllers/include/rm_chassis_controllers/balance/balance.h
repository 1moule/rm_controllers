//
// Created by yezi on 2022/11/15.
//

#pragma once

#include <rm_common/lqr.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <rm_msgs/BalanceState.h>
#include <rm_msgs/LegCmd.h>
#include <rm_common/filters/kalman_filter.h>
#include <rm_common/filters/filters.h>

#include "rm_chassis_controllers/chassis_base.h"
#include "rm_chassis_controllers/balance/helper_functions.h"

namespace rm_chassis_controllers
{
using Eigen::Matrix;
class BalanceController : public ChassisBase<rm_control::RobotStateInterface, hardware_interface::ImuSensorInterface,
                                             hardware_interface::EffortJointInterface>
{
  enum BalanceMode
  {
    NORMAL,
    STAND_UP,
    SIT_DOWN,
  };
  enum LegState
  {
    UNDER,
    FRONT,
    BEHIND,
  } left_leg_state,
      right_leg_state;

public:
  BalanceController() = default;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
  void stopping(const ros::Time& time) override;

private:
  void updateEstimation(const ros::Time& time, const ros::Duration& period);
  void detectLegState(const Eigen::Matrix<double, STATE_DIM, 1>& x, LegState& leg_state);
  void setUpLegMotion(const Eigen::Matrix<double, STATE_DIM, 1>& x, const LegState& other_leg_state,
                      const double& leg_length, const double& leg_theta, LegState& leg_state, double& theta_des,
                      double& length_des);
  void moveJoint(const ros::Time& time, const ros::Duration& period) override;
  void normal(const ros::Time& time, const ros::Duration& period);
  void standUp(const ros::Time& time, const ros::Duration& period);
  void sitDown(const ros::Time& time, const ros::Duration& period);
  geometry_msgs::Twist odometry() override;
  Eigen::Matrix<double, 4, CONTROL_DIM * STATE_DIM> coeffs_;
  Eigen::Matrix<double, STATE_DIM, STATE_DIM> q_{};
  Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM> r_{};
  Eigen::Matrix<double, STATE_DIM, 1> x_left_, x_right_;
  double vmc_bias_angle_, left_angle[2], right_angle[2], left_pos_[2], left_spd_[2], right_pos_[2], right_spd_[2];

  std::unique_ptr<ModelParams> model_params_;

  int balance_mode_;
  bool balance_state_changed_ = false;

  // stand up
  bool leg_under_body_ = false, leg_front_body_ = false, leg_behind_body_ = false, complete_stand_ = false;

  // jump
  bool complete_first_shrink_ = false, complete_elongation_ = false, complete_second_shrink_ = false;

  hardware_interface::ImuSensorHandle imu_handle_;
  hardware_interface::JointHandle left_wheel_joint_handle_, right_wheel_joint_handle_, left_first_leg_joint_handle_,
      left_second_leg_joint_handle_, right_first_leg_joint_handle_, right_second_leg_joint_handle_;

  control_toolbox::Pid pid_yaw_vel_, pid_left_leg_, pid_right_leg_, pid_theta_diff_, pid_roll_;
  control_toolbox::Pid pid_left_leg_theta_, pid_right_leg_theta_;
  control_toolbox::Pid pid_left_wheel_vel_, pid_right_wheel_vel_;

  typedef std::shared_ptr<realtime_tools::RealtimePublisher<rm_msgs::BalanceState>> RtpublisherPtr;
  RtpublisherPtr state_pub_;
  ros::Subscriber leg_cmd_sub_;
  rm_msgs::LegCmd legCmd_;
  geometry_msgs::Vector3 angular_vel_base_, linear_acc_base_;
  double roll_, pitch_, yaw_;
  double leg_length_;
};

}  // namespace rm_chassis_controllers
