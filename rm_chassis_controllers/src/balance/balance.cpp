//
// Created by qiayuan on 2022/11/15.
//
#include "rm_chassis_controllers/balance/balance.h"
#include "rm_chassis_controllers/balance/vmc/leg_conv.h"
#include "rm_chassis_controllers/balance/vmc/leg_conv_fwd.h"
#include "rm_chassis_controllers/balance/vmc/leg_pos.h"
#include "rm_chassis_controllers/balance/vmc/leg_spd.h"
#include "rm_chassis_controllers/balance/gen_A.h"
#include "rm_chassis_controllers/balance/gen_B.h"

#include <rm_common/ros_utilities.h>
#include <rm_common/ori_tool.h>
#include <rm_msgs/BalanceState.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pluginlib/class_list_macros.hpp>
#include <unsupported/Eigen/MatrixFunctions>
#include <angles/angles.h>

namespace rm_chassis_controllers
{
bool BalanceController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh,
                             ros::NodeHandle& controller_nh)
{
  ChassisBase::init(robot_hw, root_nh, controller_nh);

  imu_handle_ = robot_hw->get<hardware_interface::ImuSensorInterface>()->getHandle(
      getParam(controller_nh, "imu_name", std::string("base_imu")));
  std::string left_wheel_joint, right_wheel_joint, left_first_leg_joint, left_second_leg_joint, right_first_leg_joint,
      right_second_leg_joint;
  if (!controller_nh.getParam("left/wheel_joint", left_wheel_joint) ||
      !controller_nh.getParam("right/wheel_joint", right_wheel_joint) ||
      !controller_nh.getParam("left/first_leg_joint", left_first_leg_joint) ||
      !controller_nh.getParam("right/first_leg_joint", right_first_leg_joint) ||
      !controller_nh.getParam("left/second_leg_joint", left_second_leg_joint) ||
      !controller_nh.getParam("right/second_leg_joint", right_second_leg_joint))
  {
    ROS_ERROR("Some Joints' name doesn't given. (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  left_wheel_joint_handle_ = robot_hw->get<hardware_interface::EffortJointInterface>()->getHandle(left_wheel_joint);
  right_wheel_joint_handle_ = robot_hw->get<hardware_interface::EffortJointInterface>()->getHandle(right_wheel_joint);
  left_first_leg_joint_handle_ =
      robot_hw->get<hardware_interface::EffortJointInterface>()->getHandle(left_first_leg_joint);
  right_first_leg_joint_handle_ =
      robot_hw->get<hardware_interface::EffortJointInterface>()->getHandle(right_first_leg_joint);
  left_second_leg_joint_handle_ =
      robot_hw->get<hardware_interface::EffortJointInterface>()->getHandle(left_second_leg_joint);
  right_second_leg_joint_handle_ =
      robot_hw->get<hardware_interface::EffortJointInterface>()->getHandle(right_second_leg_joint);
  joint_handles_.push_back(left_wheel_joint_handle_);
  joint_handles_.push_back(right_wheel_joint_handle_);
  joint_handles_.push_back(left_first_leg_joint_handle_);
  joint_handles_.push_back(right_first_leg_joint_handle_);
  joint_handles_.push_back(left_second_leg_joint_handle_);
  joint_handles_.push_back(right_second_leg_joint_handle_);

  model_params_ = std::make_unique<ModelParams>();

  if (!controller_nh.getParam("m_w", model_params_->m_w))
  {
    ROS_ERROR("Params m_w doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("m_p", model_params_->m_p))
  {
    ROS_ERROR("Params m_w doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("M", model_params_->M))
  {
    ROS_ERROR("Params m doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("i_w", model_params_->i_w))
  {
    ROS_ERROR("Params i_w doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("i_m", model_params_->i_m))
  {
    ROS_ERROR("Params i_m doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("i_p", model_params_->i_p))
  {
    ROS_ERROR("Params i_m doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("l", model_params_->l))
  {
    ROS_ERROR("Params l doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("L_weight", model_params_->L_weight))
  {
    ROS_ERROR("Params l doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("Lm_weight", model_params_->Lm_weight))
  {
    ROS_ERROR("Params l doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("leg_length", leg_length_))
  {
    ROS_ERROR("Params l doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("g", model_params_->g))
  {
    ROS_ERROR("Params g doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("wheel_radius", model_params_->r))
  {
    ROS_ERROR("Params wheel_radius doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("vmc_bias_angle", vmc_bias_angle_))
  {
    ROS_ERROR("Load param fail, check the resist of vmc_bias_angle");
    return false;
  }

  if (controller_nh.hasParam("pid_yaw_vel"))
    if (!pid_yaw_vel_.init(ros::NodeHandle(controller_nh, "pid_yaw_vel")))
      return false;
  if (controller_nh.hasParam("pid_left_leg"))
    if (!pid_left_leg_.init(ros::NodeHandle(controller_nh, "pid_left_leg")))
      return false;
  if (controller_nh.hasParam("pid_right_leg"))
    if (!pid_right_leg_.init(ros::NodeHandle(controller_nh, "pid_right_leg")))
      return false;
  if (controller_nh.hasParam("pid_theta_diff"))
    if (!pid_theta_diff_.init(ros::NodeHandle(controller_nh, "pid_theta_diff")))
      return false;
  if (controller_nh.hasParam("pid_roll"))
    if (!pid_roll_.init(ros::NodeHandle(controller_nh, "pid_roll")))
      return false;
  if (controller_nh.hasParam("pid_left_leg_theta"))
    if (!pid_left_leg_theta_.init(ros::NodeHandle(controller_nh, "pid_left_leg_theta")))
      return false;
  if (controller_nh.hasParam("pid_right_leg_theta"))
    if (!pid_right_leg_theta_.init(ros::NodeHandle(controller_nh, "pid_right_leg_theta")))
      return false;
  if (controller_nh.hasParam("pid_left_wheel_vel"))
    if (!pid_left_wheel_vel_.init(ros::NodeHandle(controller_nh, "pid_left_wheel_vel")))
      return false;
  if (controller_nh.hasParam("pid_right_wheel_vel"))
    if (!pid_right_wheel_vel_.init(ros::NodeHandle(controller_nh, "pid_right_wheel_vel")))
      return false;

  q_.setZero();
  r_.setZero();
  XmlRpc::XmlRpcValue q, r;
  controller_nh.getParam("q", q);
  controller_nh.getParam("r", r);
  // Check and get Q
  ROS_ASSERT(q.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(q.size() == STATE_DIM);
  for (int i = 0; i < STATE_DIM; ++i)
  {
    ROS_ASSERT(q[i].getType() == XmlRpc::XmlRpcValue::TypeDouble || q[i].getType() == XmlRpc::XmlRpcValue::TypeInt);
    if (q[i].getType() == XmlRpc::XmlRpcValue::TypeDouble)
      q_(i, i) = static_cast<double>(q[i]);
    else if (q[i].getType() == XmlRpc::XmlRpcValue::TypeInt)
      q_(i, i) = static_cast<int>(q[i]);
  }
  // Check and get R
  ROS_ASSERT(r.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(r.size() == CONTROL_DIM);
  for (int i = 0; i < CONTROL_DIM; ++i)
  {
    ROS_ASSERT(r[i].getType() == XmlRpc::XmlRpcValue::TypeDouble || r[i].getType() == XmlRpc::XmlRpcValue::TypeInt);
    if (r[i].getType() == XmlRpc::XmlRpcValue::TypeDouble)
      r_(i, i) = static_cast<double>(r[i]);
    else if (r[i].getType() == XmlRpc::XmlRpcValue::TypeInt)
      r_(i, i) = static_cast<int>(r[i]);
  }

  // Continuous model \dot{x} = A x + B u
  std::vector<double> lengths;
  std::vector<Eigen::Matrix<double, CONTROL_DIM, STATE_DIM>> ks;
  for (int i = 5; i < 30; i++)
  {
    double length = i / 100.;
    lengths.push_back(length);
    Eigen::Matrix<double, STATE_DIM, STATE_DIM> a{};
    Eigen::Matrix<double, STATE_DIM, CONTROL_DIM> b{};
    generateAB(model_params_, a, b, length);
    Lqr<double> lqr(a, b, q_, r_);
    if (!lqr.computeK())
    {
      ROS_ERROR("Failed to compute K of LQR.");
      return false;
    }
    Eigen::Matrix<double, CONTROL_DIM, STATE_DIM> k = lqr.getK();
    ks.push_back(k);
  }
  polyfit(ks, lengths, coeffs_);

  state_pub_.reset(new realtime_tools::RealtimePublisher<rm_msgs::BalanceState>(root_nh, "/state", 100));
  auto legCmdCallback = [this](const rm_msgs::LegCmdConstPtr& msg) {
    legCmd_.leg_length = msg->leg_length;
    if (msg->jump && balance_mode_ == BalanceMode::NORMAL)
    {
      ROS_INFO("[balance] Jump start");
      legCmd_.jump = true;
    }
  };
  leg_cmd_sub_ = controller_nh.subscribe<rm_msgs::LegCmd>("/leg_command", 1, legCmdCallback);

  balance_mode_ = BalanceMode::STAND_UP;

  return true;
}

void BalanceController::updateEstimation(const ros::Time& time, const ros::Duration& period)
{
  geometry_msgs::Vector3 gyro, acc;
  gyro.x = imu_handle_.getAngularVelocity()[0];
  gyro.y = imu_handle_.getAngularVelocity()[1];
  gyro.z = imu_handle_.getAngularVelocity()[2];
  acc.x = imu_handle_.getLinearAcceleration()[0];
  acc.y = imu_handle_.getLinearAcceleration()[1];
  acc.z = imu_handle_.getLinearAcceleration()[2];
  try
  {
    tf2::doTransform(gyro, angular_vel_base_,
                     robot_state_handle_.lookupTransform("base_link", imu_handle_.getFrameId(), time));
    tf2::doTransform(acc, linear_acc_base_, robot_state_handle_.lookupTransform("odom", imu_handle_.getFrameId(), time));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    return;
  }
  tf2::Transform odom2imu, imu2base, odom2base;
  try
  {
    geometry_msgs::TransformStamped tf_msg;
    tf_msg = robot_state_handle_.lookupTransform(imu_handle_.getFrameId(), "base_link", time);
    tf2::fromMsg(tf_msg.transform, imu2base);
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    left_wheel_joint_handle_.setCommand(0.);
    right_wheel_joint_handle_.setCommand(0.);
    left_first_leg_joint_handle_.setCommand(0.);
    left_second_leg_joint_handle_.setCommand(0.);
    right_first_leg_joint_handle_.setCommand(0.);
    right_second_leg_joint_handle_.setCommand(0.);
    return;
  }
  tf2::Quaternion odom2imu_quaternion;
  tf2::Vector3 odom2imu_origin;
  odom2imu_quaternion.setValue(imu_handle_.getOrientation()[0], imu_handle_.getOrientation()[1],
                               imu_handle_.getOrientation()[2], imu_handle_.getOrientation()[3]);
  odom2imu_origin.setValue(0, 0, 0);
  odom2imu.setOrigin(odom2imu_origin);
  odom2imu.setRotation(odom2imu_quaternion);
  odom2base = odom2imu * imu2base;
  quatToRPY(toMsg(odom2base).rotation, roll_, pitch_, yaw_);

  // vmc
  // [0]:first_vmc_joint [1]:second_vmc_joint
  left_angle[0] = left_first_leg_joint_handle_.getPosition() + M_PI / 2.;
  left_angle[1] = left_second_leg_joint_handle_.getPosition() - M_PI / 4.;
  right_angle[0] = right_first_leg_joint_handle_.getPosition() + M_PI / 2.;
  right_angle[1] = right_second_leg_joint_handle_.getPosition() - M_PI / 4.;
  // [0] is length, [1] is angle
  double l1 = 0.15, l2 = 0.27;
  double xc_left = l1 * sin(left_angle[0]) + l2 * sin(left_angle[0] + left_angle[1]);
  double yc_left = l1 * cos(left_angle[0]) + l2 * cos(left_angle[0] + left_angle[1]);
  double xc_right = l1 * sin(right_angle[0]) + l2 * sin(right_angle[0] + right_angle[1]);
  double yc_right = l1 * cos(right_angle[0]) + l2 * cos(right_angle[0] + right_angle[1]);
  left_pos_[0] = sqrt(xc_left * xc_left + yc_left * yc_left);
  left_pos_[1] = (atan2(xc_left, yc_left));
  right_pos_[0] = sqrt(xc_right * xc_right + yc_right * yc_right);
  right_pos_[1] = (atan2(xc_right, yc_right));
  leg_spd(left_first_leg_joint_handle_.getVelocity(), left_second_leg_joint_handle_.getVelocity(), left_angle[0],
          left_angle[1], left_spd_);
  leg_spd(right_first_leg_joint_handle_.getVelocity(), right_second_leg_joint_handle_.getVelocity(), right_angle[0],
          right_angle[1], right_spd_);

  // update state
  x_left_[3] = (joint_handles_[0].getVelocity() + joint_handles_[1].getVelocity()) / 2.0 * wheel_radius_;
  if (abs(x_left_[3]) < 0.2 && vel_cmd_.x == 0.)
    x_left_[2] += x_left_[3] * period.toSec();
  else
    x_left_[2] = 0.;
  x_left_[0] = left_pos_[1] + pitch_;
  x_left_[1] = -left_spd_[1] + angular_vel_base_.y;
  x_left_[4] = -pitch_;
  x_left_[5] = -angular_vel_base_.y;
  x_right_ = x_left_;
  x_right_[0] = right_pos_[1] + pitch_;
  x_right_[1] = -right_spd_[1] + angular_vel_base_.y;

  if (state_pub_->trylock())
  {
    state_pub_->msg_.header.stamp = time;
    state_pub_->msg_.theta = x_left_(0);
    state_pub_->msg_.theta_dot = x_left_(1);
    state_pub_->msg_.x = x_left_(2);
    state_pub_->msg_.x_dot = x_left_(3);
    state_pub_->msg_.phi = x_left_(4);
    state_pub_->msg_.phi_dot = x_left_(5);
    state_pub_->msg_.x_b_r = x_right_(0);
    state_pub_->msg_.x_b_r_dot = x_right_(1);
    state_pub_->unlockAndPublish();
  }
}

void BalanceController::moveJoint(const ros::Time& time, const ros::Duration& period)
{
  updateEstimation(time, period);
  switch (balance_mode_)
  {
    case BalanceMode::NORMAL:
    {
      normal(time, period);
      break;
    }
    case BalanceMode::STAND_UP:
    {
      standUp(time, period);
      break;
    }
    case BalanceMode::SIT_DOWN:
    {
      sitDown(time, period);
      break;
    }
  }
}

void BalanceController::normal(const ros::Time& time, const ros::Duration& period)
{
  if (!balance_state_changed_)
  {
    ROS_INFO("[balance] Enter NOMAl");
    balance_state_changed_ = true;
  }
  if (!complete_stand_ && abs(x_left_[4]) < 0.2)
    complete_stand_ = true;

  // PID
  double T_yaw = pid_yaw_vel_.computeCommand(vel_cmd_.z - angular_vel_base_.z, period);
  double T_theta_diff = pid_theta_diff_.computeCommand(left_pos_[1] - right_pos_[1], period);
  double T_roll = pid_roll_.computeCommand(0. - roll_, period);

  // LQR
  Eigen::Matrix<double, CONTROL_DIM, STATE_DIM> k_left{}, k_right{};
  for (int i = 0; i < 2; i++)
    for (int j = 0; j < 6; j++)
    {
      k_left(i, j) = coeffs_(0, i + 2 * j) * pow(left_pos_[0], 3) + coeffs_(1, i + 2 * j) * pow(left_pos_[0], 2) +
                     coeffs_(2, i + 2 * j) * left_pos_[0] + coeffs_(3, i + 2 * j);
      k_right(i, j) = coeffs_(0, i + 2 * j) * pow(right_pos_[0], 3) + coeffs_(1, i + 2 * j) * pow(right_pos_[0], 2) +
                      coeffs_(2, i + 2 * j) * right_pos_[0] + coeffs_(3, i + 2 * j);
    }
  Eigen::Matrix<double, CONTROL_DIM, 1> u_left, u_right;
  auto x_left = x_left_;
  auto x_right = x_right_;
  if (complete_stand_)
  {
    x_left(3) -= vel_cmd_.x;
    x_right(3) -= vel_cmd_.x;
  }
  u_left = k_left * (-x_left);
  u_right = k_right * (-x_right);

  // Leg control
  double gravity = 1. / 2. * model_params_->M * model_params_->g;
  Eigen::Matrix<double, 2, 1> F_leg;
  double leg_length_des = leg_length_;
  if (!start_jump_ && legCmd_.jump && abs(x_left[0]) < 0.1)
    start_jump_ = true;
  if (start_jump_)
  {
    if (!complete_first_shrink_)
      leg_length_des = 0.15;
    if (!complete_first_shrink_ && abs(0.15 - left_pos_[0]) < 0.02)
      complete_first_shrink_ = true;
    if (complete_first_shrink_ && !complete_elongation_)
      leg_length_des = 0.4;
    if (complete_first_shrink_ && !complete_elongation_ && abs(0.4 - left_pos_[0]) < 0.02)
      complete_elongation_ = true;
    if (complete_elongation_ && !complete_second_shrink_)
      leg_length_des = 0.15;
    if (complete_elongation_ && !complete_second_shrink_ && abs(0.15 - left_pos_[0]) < 0.02)
      complete_second_shrink_ = true;
    if (complete_second_shrink_)
    {
      complete_first_shrink_ = false;
      complete_elongation_ = false;
      complete_second_shrink_ = false;
      legCmd_.jump = false;
      start_jump_ = false;
      ROS_INFO("[balance] Jump finished");
    }
    F_leg[0] =
        pid_left_leg_.computeCommand(leg_length_des - left_pos_[0], period) + gravity * cos(left_pos_[1]) + T_roll;
    F_leg[1] =
        pid_right_leg_.computeCommand(leg_length_des - right_pos_[0], period) + gravity * cos(right_pos_[1]) - T_roll;
  }
  else
  {
    double left_length_des = complete_stand_ ? legCmd_.leg_length / cos(x_left[0]) : 0.18;
    double right_length_des = complete_stand_ ? legCmd_.leg_length / cos(x_right[0]) : 0.18;
    F_leg[0] =
        pid_left_leg_.computeCommand(left_length_des - left_pos_[0], period) + gravity * cos(left_pos_[1]) + T_roll;
    F_leg[1] =
        pid_right_leg_.computeCommand(right_length_des - right_pos_[0], period) + gravity * cos(right_pos_[1]) - T_roll;
  }
  double left_T[2], right_T[2];
  leg_conv(F_leg[0], -u_left(1) + T_theta_diff, left_angle[0], left_angle[1], left_T);
  leg_conv(F_leg[1], -u_right(1) - T_theta_diff, right_angle[0], right_angle[1], right_T);

  // Unstick detection
  bool maybe_unstick = false, unstick = false;
  double left_F[2], right_F[2];
  leg_conv_fwd(left_first_leg_joint_handle_.getEffort(), left_second_leg_joint_handle_.getEffort(), left_angle[0],
               left_angle[1], left_F);
  leg_conv_fwd(right_first_leg_joint_handle_.getEffort(), right_second_leg_joint_handle_.getEffort(), right_angle[0],
               right_angle[1], right_F);
  Eigen::Matrix<double, CONTROL_DIM, 1> u_left_real, u_right_real;
  u_left_real << left_wheel_joint_handle_.getEffort(), left_F[1];
  u_right_real << right_wheel_joint_handle_.getEffort(), right_F[1];
  double Fn_left = calculateSupportForce(left_F[0], left_F[1], left_pos_[0], linear_acc_base_.z, x_left_, u_left_real,
                                         model_params_);
  double Fn_right = calculateSupportForce(right_F[0], right_F[1], right_pos_[0], linear_acc_base_.z, x_right_,
                                          u_right_real, model_params_);
  Eigen::Matrix<double, CONTROL_DIM, STATE_DIM> k_left_unstick{}, k_right_unstick{};
  k_left_unstick.setZero();
  k_right_unstick.setZero();
  k_left_unstick(1, 0) = k_left(1, 0);
  k_right_unstick(1, 1) = k_right(1, 1);
  if (Fn_left < 20. && complete_stand_)
  {
    u_left = k_left_unstick * (-x_left);
    leg_conv(F_leg[0], -u_left(1) + T_theta_diff, left_angle[0], left_angle[1], left_T);
    maybe_unstick = true;
  }
  if (Fn_right < 20. && complete_stand_)
  {
    u_right = k_right_unstick * (-x_right);
    leg_conv(F_leg[1], -u_right(1) - T_theta_diff, right_angle[0], right_angle[1], right_T);
    unstick = maybe_unstick ? true : false;
  }

  // control
  if (complete_stand_ && (abs(x_left(4)) > 0.4 || abs(x_left(0)) > 1.5))
  {
    balance_mode_ = BalanceMode::SIT_DOWN;
    balance_state_changed_ = false;
    complete_first_shrink_ = false;
    complete_elongation_ = false;
    complete_second_shrink_ = false;
    legCmd_.jump = false;
    left_wheel_joint_handle_.setCommand(0.);
    right_wheel_joint_handle_.setCommand(0.);
    left_first_leg_joint_handle_.setCommand(0.);
    left_second_leg_joint_handle_.setCommand(0.);
    right_first_leg_joint_handle_.setCommand(0.);
    right_second_leg_joint_handle_.setCommand(0.);
    ROS_INFO("[balance] Exit NORMAL");
  }
  else
  {
    left_wheel_joint_handle_.setCommand(unstick ? 0. : u_left(0) - T_yaw);
    right_wheel_joint_handle_.setCommand(unstick ? 0. : u_right(0) + T_yaw);
    left_first_leg_joint_handle_.setCommand(left_T[0]);
    right_first_leg_joint_handle_.setCommand(right_T[0]);
    left_second_leg_joint_handle_.setCommand(left_T[1]);
    right_second_leg_joint_handle_.setCommand(right_T[1]);
  }
}

void BalanceController::detectLegState(const Eigen::Matrix<double, STATE_DIM, 1>& x, LegState& leg_state)
{
  if (x[0] > -M_PI / 2 + 0.1 && x[0] < M_PI / 2 - 0.2)
    leg_state = LegState::UNDER;
  else if (x[0] < -M_PI / 2 + 0.1 && x[0] > -M_PI)
    leg_state = LegState::FRONT;
  else if (x[0] > M_PI / 2 - 0.2 && x[0] < M_PI)
    leg_state = LegState::BEHIND;
}

void BalanceController::setUpLegMotion(const Eigen::Matrix<double, STATE_DIM, 1>& x, const LegState& other_leg_state,
                                       const double& leg_length, const double& leg_theta, LegState& leg_state,
                                       double& theta_des, double& length_des)
{
  switch (leg_state)
  {
    case LegState::UNDER:
      theta_des = 0.15;
      length_des = 0.05;
      break;
    case LegState::FRONT:
      theta_des = M_PI / 2 + 0.2;
      length_des = 0.4;
      if (abs(angles::shortest_angular_distance(x[0], M_PI / 2)) < 0.2 && abs(x[4]) < 0.1)
        leg_state = LegState::BEHIND;
      break;
    case LegState::BEHIND:
      theta_des = leg_theta;
      length_des = leg_length;
      if (other_leg_state != LegState::FRONT)
      {
        theta_des = 0.;
        length_des = 0.05;
      }
      break;
  }
}

void BalanceController::standUp(const ros::Time& time, const ros::Duration& period)
{
  if (!balance_state_changed_)
  {
    ROS_INFO("[balance] Enter STAND_UP");
    balance_state_changed_ = true;
    complete_stand_ = false;
    detectLegState(x_left_, left_leg_state);
    detectLegState(x_right_, right_leg_state);
  }
  Eigen::Matrix<double, 2, 1> F_leg;
  double T_theta_l, T_theta_r, theta_des_l, theta_des_r, length_des_l, length_des_r;
  setUpLegMotion(x_left_, right_leg_state, left_pos_[0], left_pos_[1], left_leg_state, theta_des_l, length_des_l);
  setUpLegMotion(x_right_, left_leg_state, right_pos_[0], right_pos_[1], right_leg_state, theta_des_r, length_des_r);
  F_leg[0] = pid_left_leg_.computeCommand(length_des_l - left_pos_[0], period);
  F_leg[1] = pid_right_leg_.computeCommand(length_des_r - right_pos_[0], period);
  T_theta_l = pid_left_leg_theta_.computeCommand(-angles::shortest_angular_distance(theta_des_l, left_pos_[1]), period);
  T_theta_r =
      pid_right_leg_theta_.computeCommand(-angles::shortest_angular_distance(theta_des_r, right_pos_[1]), period);
  double left_T[2], right_T[2];
  leg_conv(F_leg[0], -T_theta_l, left_angle[0], left_angle[1], left_T);
  leg_conv(F_leg[1], -T_theta_r, right_angle[0], right_angle[1], right_T);
  left_first_leg_joint_handle_.setCommand(left_T[0]);
  right_first_leg_joint_handle_.setCommand(right_T[0]);
  left_second_leg_joint_handle_.setCommand(left_T[1]);
  right_second_leg_joint_handle_.setCommand(right_T[1]);
  left_wheel_joint_handle_.setCommand(0.);
  right_wheel_joint_handle_.setCommand(0.);
  if (((left_pos_[1] < 0. && left_leg_state == LegState::BEHIND) ||
       (left_pos_[1] > 0. && left_leg_state == LegState::UNDER)) &&
      ((right_pos_[1] < 0. && right_leg_state == LegState::BEHIND) ||
       (right_pos_[1] > 0. && right_leg_state == LegState::UNDER)))
  {
    balance_mode_ = NORMAL;
    balance_state_changed_ = false;
    ROS_INFO("[balance] Exit STAND_UP");
  }
}

void BalanceController::sitDown(const ros::Time& time, const ros::Duration& period)
{
  if (!balance_state_changed_)
  {
    ROS_INFO("[balance] Enter SIT_DOWN");
    balance_state_changed_ = true;
  }
  left_wheel_joint_handle_.setCommand(pid_left_wheel_vel_.computeCommand(-joint_handles_[0].getVelocity(), period));
  right_wheel_joint_handle_.setCommand(pid_right_wheel_vel_.computeCommand(-joint_handles_[1].getVelocity(), period));
  left_first_leg_joint_handle_.setCommand(0.);
  left_second_leg_joint_handle_.setCommand(0.);
  right_first_leg_joint_handle_.setCommand(0.);
  right_second_leg_joint_handle_.setCommand(0.);
  if (abs(x_left_(1)) < 0.1 && abs(x_left_(5)) < 0.1 && abs(x_left_(3)) < 0.1)
  {
    balance_mode_ = BalanceMode::STAND_UP;
    balance_state_changed_ = false;
    ROS_INFO("[balance] Exit NORMAL");
  }
}

geometry_msgs::Twist BalanceController::odometry()
{
  geometry_msgs::Twist twist;
  twist.linear.x = x_left_[3];
  return twist;
}

void BalanceController::stopping(const ros::Time& time)
{
  balance_mode_ = BalanceMode::STAND_UP;
  balance_state_changed_ = false;
}
}  // namespace rm_chassis_controllers
PLUGINLIB_EXPORT_CLASS(rm_chassis_controllers::BalanceController, controller_interface::ControllerBase)
