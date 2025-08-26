//
// Created by qiayuan on 2022/11/15.
//
#include "rm_chassis_controllers/balance/balance.h"
#include "rm_chassis_controllers/balance/vmc/leg_conv.h"
#include "rm_chassis_controllers/balance/vmc/leg_pos.h"
#include "rm_chassis_controllers/balance/vmc/leg_spd.h"
#include "rm_chassis_controllers/balance/gen_A.h"
#include "rm_chassis_controllers/balance/gen_B.h"

#include <unsupported/Eigen/MatrixFunctions>
#include "rm_common/ros_utilities.h"
#include "rm_common/ori_tool.h"
#include <geometry_msgs/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pluginlib/class_list_macros.hpp>
#include <rm_msgs/BalanceState.h>
#include <angles/angles.h>

namespace rm_chassis_controllers
{
bool BalanceController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh,
                             ros::NodeHandle& controller_nh)
{
  ChassisBase::init(robot_hw, root_nh, controller_nh);

  imu_handle_ = robot_hw->get<hardware_interface::ImuSensorInterface>()->getHandle(
      getParam(controller_nh, "imu_name", std::string("base_imu")));
  std::string left_wheel_joint, right_wheel_joint, left_front_leg_joint, left_back_leg_joint, right_front_leg_joint,
      right_back_leg_joint;
  if (!controller_nh.getParam("left/wheel_joint", left_wheel_joint) ||
      !controller_nh.getParam("right/wheel_joint", right_wheel_joint) ||
      !controller_nh.getParam("left/front_leg_joint", left_front_leg_joint) ||
      !controller_nh.getParam("right/front_leg_joint", right_front_leg_joint) ||
      !controller_nh.getParam("left/back_leg_joint", left_back_leg_joint) ||
      !controller_nh.getParam("right/back_leg_joint", right_back_leg_joint))
  {
    ROS_ERROR("Some Joints' name doesn't given. (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  left_wheel_joint_handle_ = robot_hw->get<hardware_interface::EffortJointInterface>()->getHandle(left_wheel_joint);
  right_wheel_joint_handle_ = robot_hw->get<hardware_interface::EffortJointInterface>()->getHandle(right_wheel_joint);
  left_front_leg_joint_handle_ =
      robot_hw->get<hardware_interface::EffortJointInterface>()->getHandle(left_front_leg_joint);
  right_front_leg_joint_handle_ =
      robot_hw->get<hardware_interface::EffortJointInterface>()->getHandle(right_front_leg_joint);
  left_back_leg_joint_handle_ =
      robot_hw->get<hardware_interface::EffortJointInterface>()->getHandle(left_back_leg_joint);
  right_back_leg_joint_handle_ =
      robot_hw->get<hardware_interface::EffortJointInterface>()->getHandle(right_back_leg_joint);
  joint_handles_.push_back(left_wheel_joint_handle_);
  joint_handles_.push_back(right_wheel_joint_handle_);
  joint_handles_.push_back(left_front_leg_joint_handle_);
  joint_handles_.push_back(right_front_leg_joint_handle_);
  joint_handles_.push_back(left_back_leg_joint_handle_);
  joint_handles_.push_back(right_back_leg_joint_handle_);

  if (!controller_nh.getParam("m_w", m_w_))
  {
    ROS_ERROR("Params m_w doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("m_p", m_p))
  {
    ROS_ERROR("Params m_w doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("M", M))
  {
    ROS_ERROR("Params m doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("i_w", i_w))
  {
    ROS_ERROR("Params i_w doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("i_m", i_m))
  {
    ROS_ERROR("Params i_m doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("i_p", i_p))
  {
    ROS_ERROR("Params i_m doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("l", l))
  {
    ROS_ERROR("Params l doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("L_weight", L_weight))
  {
    ROS_ERROR("Params l doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("Lm_weight", Lm_weight))
  {
    ROS_ERROR("Params l doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("leg_length", leg_length_))
  {
    ROS_ERROR("Params l doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("g", g_))
  {
    ROS_ERROR("Params g doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("wheel_radius", wheel_radius_))
  {
    ROS_ERROR("Params wheel_radius doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("vmc_bias_angle", vmc_bias_angle_))
  {
    ROS_ERROR("Load param fail, check the resist of vmc_bias_angle");
    return false;
  }
  body_mass_ = M;

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
  double A[36]{ 0. }, B[12]{ 0. };
  std::vector<double> lengths;
  std::vector<Eigen::Matrix<double, CONTROL_DIM, STATE_DIM>> ks;
  for (int i = 10; i < 30; i++)
  {
    double length = i / 100.;
    lengths.push_back(length);
    L = length * L_weight;
    Lm = length * Lm_weight;
    gen_A(i_m, i_p, i_w, L, Lm, M, wheel_radius_, g_, l, m_p, m_w_, A);
    gen_B(i_m, i_p, i_w, L, Lm, M, wheel_radius_, l, m_p, m_w_, B);
    Eigen::Matrix<double, STATE_DIM, STATE_DIM> a{};
    Eigen::Matrix<double, STATE_DIM, CONTROL_DIM> b{};
    // clang-format off
    a<< 0.  ,1.,0.,0.,0.   ,0.,
        A[1],0.,0.,0.,A[25],0.,
        0.  ,0.,0.,1.,0.   ,0.,
        A[3],0.,0.,0.,A[27],0.,
        0.  ,0.,0.,0.,0.   ,1.,
        A[5],0.,0.,0.,A[29],0.;
    b<< 0.  ,0.  ,
        B[1],B[7],
        0.  ,0.  ,
        B[3],B[9],
        0.  ,0.  ,
        B[5],B[11];
    // clang-format on
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
    if (msg->jump)
    {
      ROS_INFO("[balance] Jump start");
      legCmd_.jump = true;
    }
  };
  leg_cmd_sub_ = controller_nh.subscribe<rm_msgs::LegCmd>("/leg_command", 1, legCmdCallback);

  balance_mode_ = BalanceMode::NORMAL;

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
    left_front_leg_joint_handle_.setCommand(0.);
    left_back_leg_joint_handle_.setCommand(0.);
    right_front_leg_joint_handle_.setCommand(0.);
    right_back_leg_joint_handle_.setCommand(0.);
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
  // [0]:back_vmc_joint [1]:front_vmc_joint
  left_angle[0] = vmc_bias_angle_ + left_back_leg_joint_handle_.getPosition();
  left_angle[1] = left_front_leg_joint_handle_.getPosition() + 3.1415926 - vmc_bias_angle_;
  right_angle[0] = vmc_bias_angle_ + right_back_leg_joint_handle_.getPosition();
  right_angle[1] = right_front_leg_joint_handle_.getPosition() + 3.1415926 - vmc_bias_angle_;
  leg_pos(left_angle[0], left_angle[1], left_pos_);
  leg_pos(right_angle[0], right_angle[1], right_pos_);
  leg_spd(left_back_leg_joint_handle_.getVelocity(), left_front_leg_joint_handle_.getVelocity(), left_angle[0],
          left_angle[1], left_spd_);
  leg_spd(right_back_leg_joint_handle_.getVelocity(), right_front_leg_joint_handle_.getVelocity(), right_angle[0],
          right_angle[1], right_spd_);

  // update state
  x_left_[3] = (joint_handles_[0].getVelocity() + joint_handles_[1].getVelocity()) / 2.0 * wheel_radius_;
  if (abs(x_left_[3]) < 0.1)
    x_left_[2] += x_left_[3] * period.toSec();
  else
    x_left_[2] = 0.;
  x_left_[0] = left_pos_[1] + pitch_;
  x_left_[1] = left_spd_[1] + angular_vel_base_.y;
  x_left_[4] = -pitch_;
  x_left_[5] = -angular_vel_base_.y;
  x_right_ = x_left_;
  x_right_[0] = right_pos_[1] + pitch_;
  x_right_[1] = right_spd_[1] + angular_vel_base_.y;
}

double BalanceController::unstickDetection(const ros::Time& time, const ros::Duration& period, double F, double Tp,
                                           double leg_length, Eigen::Matrix<double, STATE_DIM, 1> x,
                                           Eigen::Matrix<double, CONTROL_DIM, 1> u)
{
  double A[36] = { 0. }, B[12]{ 0. };
  L = leg_length * L_weight;
  Lm = leg_length * Lm_weight;
  gen_A(i_m, i_p, i_w, L, Lm, M, wheel_radius_, g_, l, m_p, m_w_, A);
  gen_B(i_m, i_p, i_w, L, Lm, M, wheel_radius_, l, m_p, m_w_, B);
  Eigen::Matrix<double, STATE_DIM, STATE_DIM> a{};
  Eigen::Matrix<double, STATE_DIM, CONTROL_DIM> b{};
  // clang-format off
  a<< 0.  ,1.,0.,0.,0.   ,0.,
      A[1],0.,0.,0.,A[25],0.,
      0.  ,0.,0.,1.,0.   ,0.,
      A[3],0.,0.,0.,A[27],0.,
      0.  ,0.,0.,0.,0.   ,1.,
      A[5],0.,0.,0.,A[29],0.;
  b<< 0.  ,0.  ,
      B[1],B[7],
      0.  ,0.  ,
      B[3],B[9],
      0.  ,0.  ,
      B[5],B[11];
  // clang-format on

  double P = F * cos(x(0)) + Tp * sin(x(0)) / leg_length;
  double ddot_zM = linear_acc_base_.z - g_;
  auto ddot_x = a * x + b * u;
  double ddot_theta = ddot_x(1);
  double ddot_zw = ddot_zM - leg_length * cos(x(0)) + 2 * leg_length * x(1) * sin(x(0)) +
                   +leg_length * (ddot_theta * sin(x(0)) + x(1) * x(1) * cos(x(0)));
  double Fn = m_w_ * ddot_zw + m_w_ * g_ + P;
  return Fn;
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
  }
}

void BalanceController::normal(const ros::Time& time, const ros::Duration& period)
{
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
  x_left(3) -= vel_cmd_.x;
  x_right(3) -= vel_cmd_.x;
  u_left = k_left * (-x_left);
  u_right = k_right * (-x_right);

  // Leg control
  double gravity = 1. / 2. * body_mass_ * g_;
  Eigen::Matrix<double, 2, 1> F_leg;
  double leg_length_des = leg_length_;
  if (legCmd_.jump)
  {
    if (!complete_first_shrink_)
      leg_length_des = 0.1;
    if (!complete_first_shrink_ && abs(0.1 - left_pos_[0]) < 0.02)
      complete_first_shrink_ = true;
    if (complete_first_shrink_ && !complete_elongation_)
      leg_length_des = 0.35;
    if (complete_first_shrink_ && !complete_elongation_ && abs(0.35 - left_pos_[0]) < 0.02)
      complete_elongation_ = true;
    if (complete_elongation_ && !complete_second_shrink_)
      leg_length_des = 0.1;
    if (complete_elongation_ && !complete_second_shrink_ && abs(0.1 - left_pos_[0]) < 0.02)
      complete_second_shrink_ = true;
    if (complete_second_shrink_)
    {
      complete_first_shrink_ = false;
      complete_elongation_ = false;
      complete_second_shrink_ = false;
      legCmd_.jump = false;
      ROS_INFO("[balance] Jump finished");
    }
    F_leg[0] =
        pid_left_leg_.computeCommand(leg_length_des - left_pos_[0], period) + gravity * cos(left_pos_[1]) + T_roll;
    F_leg[1] =
        pid_right_leg_.computeCommand(leg_length_des - right_pos_[0], period) + gravity * cos(right_pos_[1]) - T_roll;
  }
  else
  {
    F_leg[0] = pid_left_leg_.computeCommand(legCmd_.leg_length / cos(x_left(0)) - left_pos_[0], period) +
               gravity * cos(left_pos_[1]) + T_roll;
    F_leg[1] = pid_right_leg_.computeCommand(legCmd_.leg_length / cos(x_right(0)) - right_pos_[0], period) +
               gravity * cos(right_pos_[1]) - T_roll;
  }
  double left_T[2], right_T[2];
  leg_conv(F_leg[0], u_left(1) - T_theta_diff, left_angle[0], left_angle[1], left_T);
  leg_conv(F_leg[1], u_right(1) + T_theta_diff, right_angle[0], right_angle[1], right_T);

  // Unstick detection
  double Fn_left = unstickDetection(time, period, F_leg[0], u_left(1) - T_theta_diff, left_pos_[0], x_left_, u_left);
  double Fn_right =
      unstickDetection(time, period, F_leg[1], u_right(1) + T_theta_diff, right_pos_[0], x_right_, u_right);
  Eigen::Matrix<double, CONTROL_DIM, STATE_DIM> k_left_unstick{}, k_right_unstick{};
  k_left_unstick.setZero();
  k_right_unstick.setZero();
  k_left_unstick(1, 0) = k_left(1, 0);
  k_right_unstick(1, 1) = k_right(1, 1);
  if (Fn_left < 10. && !legCmd_.jump)
  {
    u_left = k_left_unstick * (-x_left);
    leg_conv(0., u_left(1) - T_theta_diff, left_angle[0], left_angle[1], left_T);
  }
  if (Fn_right < 10. && !legCmd_.jump)
  {
    u_right = k_right_unstick * (-x_right);
    leg_conv(0., u_right(1) + T_theta_diff, right_angle[0], right_angle[1], right_T);
  }

  left_wheel_joint_handle_.setCommand(u_left(0) - T_yaw);
  right_wheel_joint_handle_.setCommand(u_right(0) + T_yaw);
  left_front_leg_joint_handle_.setCommand(left_T[1]);
  right_front_leg_joint_handle_.setCommand(right_T[1]);
  left_back_leg_joint_handle_.setCommand(left_T[0]);
  right_back_leg_joint_handle_.setCommand(right_T[0]);

  if (state_pub_->trylock())
  {
    state_pub_->msg_.header.stamp = time;
    state_pub_->msg_.theta = x_left(0);
    state_pub_->msg_.theta_dot = x_left(1);
    state_pub_->msg_.x = x_left(2);
    state_pub_->msg_.x_dot = x_left(3);
    state_pub_->msg_.phi = x_left(4);
    state_pub_->msg_.phi_dot = x_left(5);
    state_pub_->msg_.x_b_r = x_right(0);
    state_pub_->msg_.x_b_r_dot = x_right(1);
    state_pub_->msg_.f_b_l = left_pos_[0];
    state_pub_->msg_.f_b_r = right_pos_[0];
    state_pub_->msg_.T_l = u_left(1);
    state_pub_->msg_.T_r = u_right(1);
    state_pub_->unlockAndPublish();
  }
}

inline void BalanceController::polyfit(const std::vector<Eigen::Matrix<double, 2, 6>>& Ks,
                                       const std::vector<double>& L0s, Eigen::Matrix<double, 4, 12>& coeffs)
{
  int N = L0s.size();
  Eigen::MatrixXd A(N, 4), B(N, 12);
  for (int i = 0; i < N; ++i)
  {
    A.block(i, 0, 1, 4) << pow(L0s[i], 3), pow(L0s[i], 2), L0s[i], 1.0;
    Eigen::Map<const Eigen::Matrix<double, 12, 1>> flat(Ks[i].data());
    B.row(i) = flat.transpose();
  }
  coeffs = (A.transpose() * A).ldlt().solve(A.transpose() * B);
}

geometry_msgs::Twist BalanceController::odometry()
{
  geometry_msgs::Twist twist;
  twist.linear.x = x_left_[3];
  return twist;
}
}  // namespace rm_chassis_controllers
PLUGINLIB_EXPORT_CLASS(rm_chassis_controllers::BalanceController, controller_interface::ControllerBase)
