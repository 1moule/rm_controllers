//
// Created by qiayuan on 2022/11/15.
//
#include "rm_chassis_controllers/balance.h"

#include <unsupported/Eigen/MatrixFunctions>
#include <rm_common/ros_utilities.h>
#include <rm_common/ori_tool.h>
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

  // m_w is mass of single wheel
  // m is mass of the robot except wheels and momentum_blocks
  // i_w is the moment of inertia of the wheel around the rotational axis of the motor
  // i_m is the moment of inertia of the robot around the y-axis of base_link coordinate.
  // l is the vertical component of the distance between the wheel center and the center of mass of robot
  //  double m_w, m, i_w, i_m, l, g;
  double L, Lm, l, m_w, m_p, M, i_w, i_p, i_m, g;
  double leg_length;

  if (!controller_nh.getParam("m_w", m_w))
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
  if (!controller_nh.getParam("L", L))
  {
    ROS_ERROR("Params l doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("Lm", Lm))
  {
    ROS_ERROR("Params l doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("leg_length", leg_length))
  {
    ROS_ERROR("Params l doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("g", g))
  {
    ROS_ERROR("Params g doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("wheel_radius", wheel_radius_))
  {
    ROS_ERROR("Params wheel_radius doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  L = leg_length * 0.75;
  Lm = leg_length * 0.25;

  if (controller_nh.hasParam("pid_yaw_vel"))
    if (!pid_yaw_vel_.init(ros::NodeHandle(controller_nh, "pid_yaw_vel")))
      return false;
  if (controller_nh.hasParam("pid_vel_x"))
    if (!pid_vel_x_.init(ros::NodeHandle(controller_nh, "pid_vel_x")))
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
  generateA(wheel_radius_, l, Lm, l, m_w, m_p, M, i_w, i_p, i_m, g, a_);
  generateB(wheel_radius_, l, Lm, l, m_w, m_p, M, i_w, i_p, i_m, g, b_);

  ROS_INFO_STREAM("A:" << a_);
  ROS_INFO_STREAM("B:" << b_);
  Lqr<double> lqr(a_, b_, q_, r_);
  if (!lqr.computeK())
  {
    ROS_ERROR("Failed to compute K of LQR.");
    return false;
  }

  k_ = lqr.getK();
  ROS_INFO_STREAM("K of LQR:" << k_);

  state_pub_.reset(new realtime_tools::RealtimePublisher<rm_msgs::BalanceState>(root_nh, "/state", 100));
  balance_mode_ = BalanceMode::NORMAL;

  return true;
}

void BalanceController::moveJoint(const ros::Time& time, const ros::Duration& period)
{
  geometry_msgs::Vector3 gyro;
  gyro.x = imu_handle_.getAngularVelocity()[0];
  gyro.y = imu_handle_.getAngularVelocity()[1];
  gyro.z = imu_handle_.getAngularVelocity()[2];
  try
  {
    tf2::doTransform(gyro, angular_vel_base_,
                     robot_state_handle_.lookupTransform("base_link", imu_handle_.getFrameId(), time));
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
  x_[0] = pitch_;
  x_[1] = angular_vel_base_.y;
  x_[3] = (left_wheel_joint_handle_.getVelocity() + right_wheel_joint_handle_.getVelocity()) / 2. * wheel_radius_;
  x_[2] += x_[3] * period.toSec();
  Eigen::Matrix<double, CONTROL_DIM, 1> u;
  auto x = x_;
  x(3) -= vel_cmd_.x;
  u = k_ * (-x);
  pid_yaw_vel_.computeCommand(vel_cmd_.z - angular_vel_base_.z, period);
  pid_vel_x_.computeCommand(vel_cmd_.x - x_[3], period);
  if (state_pub_->trylock())
  {
    state_pub_->msg_.header.stamp = time;
    state_pub_->msg_.theta = x(0);
    state_pub_->msg_.theta_dot = x(1);
    state_pub_->msg_.x = x(2);
    state_pub_->msg_.x_dot = x(3);
    state_pub_->msg_.T_l = u(0);
    state_pub_->msg_.T_r = u(0);
    state_pub_->unlockAndPublish();
  }

  left_wheel_joint_handle_.setCommand(u(0) - pid_yaw_vel_.getCurrentCmd() + pid_vel_x_.getCurrentCmd());
  right_wheel_joint_handle_.setCommand(u(0) + pid_yaw_vel_.getCurrentCmd() + pid_vel_x_.getCurrentCmd());
}

geometry_msgs::Twist BalanceController::odometry()
{
  geometry_msgs::Twist twist;
  twist.linear.x = x_[3];
  return twist;
}

void BalanceController::generateA(double R, double L, double Lm, double l, double mw, double mp, double M, double Iw,
                                  double Ip, double Im, double g, Eigen::Matrix<double, STATE_DIM, STATE_DIM>& a)
{
  double a_2_1 =
      (Im * Iw * L * g * mp + Im * L * M * M * R * R * g + Im * Lm * M * M * R * R * g + Iw * L * M * M * g * l * l +
       Iw * Lm * M * M * g * l * l + Im * L * R * R * g * mp * mp + Im * Iw * L * M * g + Im * Iw * Lm * M * g +
       L * M * R * R * g * l * l * mp * mp + L * M * M * R * R * g * l * l * mp + L * M * M * R * R * g * l * l * mw +
       Lm * M * M * R * R * g * l * l * mp + Lm * M * M * R * R * g * l * l * mw + 2.0 * Im * L * M * R * R * g * mp +
       Im * L * M * R * R * g * mw + Im * Lm * M * R * R * g * mp + Im * Lm * M * R * R * g * mw +
       Iw * L * M * g * l * l * mp + Im * L * R * R * g * mp * mw + L * M * R * R * g * l * l * mp * mw) /
      (Im * Ip * Iw + Ip * Iw * M * l * l + Im * Iw * L * L * mp + Im * Ip * R * R * mp + Im * Ip * R * R * mw +
       Im * Iw * L * L * M + Im * Iw * Lm * Lm * M + Im * Ip * M * R * R + Im * L * L * M * R * R * mw +
       Im * Lm * Lm * M * R * R * mp + Im * Lm * Lm * M * R * R * mw + 2.0 * Im * Iw * L * Lm * M +
       Iw * L * L * M * l * l * mp + Ip * M * R * R * l * l * mp + Ip * M * R * R * l * l * mw +
       Im * L * L * R * R * mp * mw + 2.0 * Im * L * Lm * M * R * R * mw + L * L * M * R * R * l * l * mp * mw);
  double a_4_1 =
      -(g * L * L * M * M * R * R * l * l * mp + g * Im * L * L * M * M * R * R +
        g * L * L * M * R * R * l * l * mp * mp + 2.0 * g * Im * L * L * M * R * R * mp +
        g * Im * L * L * R * R * mp * mp + g * L * Lm * M * M * R * R * l * l * mp +
        2.0 * g * Im * L * Lm * M * M * R * R + 2.0 * g * Im * L * Lm * M * R * R * mp +
        g * Im * Lm * Lm * M * M * R * R) /
      (Im * Ip * Iw + Ip * Iw * M * l * l + Im * Iw * L * L * mp + Im * Ip * R * R * mp + Im * Ip * R * R * mw +
       Im * Iw * L * L * M + Im * Iw * Lm * Lm * M + Im * Ip * M * R * R + Im * L * L * M * R * R * mw +
       Im * Lm * Lm * M * R * R * mp + Im * Lm * Lm * M * R * R * mw + 2.0 * Im * Iw * L * Lm * M +
       Iw * L * L * M * l * l * mp + Ip * M * R * R * l * l * mp + Ip * M * R * R * l * l * mw +
       Im * L * L * R * R * mp * mw + 2.0 * Im * L * Lm * M * R * R * mw + L * L * M * R * R * l * l * mp * mw);
  double a_6_1 =
      (g * l * mw * L * L * M * M * R * R + Iw * g * l * L * L * M * M + g * l * mw * L * L * M * R * R * mp +
       Iw * g * l * L * L * M * mp + g * l * L * Lm * M * M * R * R * mp + 2.0 * g * l * mw * L * Lm * M * M * R * R +
       2.0 * Iw * g * l * L * Lm * M * M + g * l * L * Lm * M * R * R * mp * mp + g * l * mw * L * Lm * M * R * R * mp +
       Iw * g * l * L * Lm * M * mp + g * l * Lm * Lm * M * M * R * R * mp + g * l * mw * Lm * Lm * M * M * R * R +
       Iw * g * l * Lm * Lm * M * M) /
      (Im * Ip * Iw + Ip * Iw * M * l * l + Im * Iw * L * L * mp + Im * Ip * R * R * mp + Im * Ip * R * R * mw +
       Im * Iw * L * L * M + Im * Iw * Lm * Lm * M + Im * Ip * M * R * R + Im * L * L * M * R * R * mw +
       Im * Lm * Lm * M * R * R * mp + Im * Lm * Lm * M * R * R * mw + 2.0 * Im * Iw * L * Lm * M +
       Iw * L * L * M * l * l * mp + Ip * M * R * R * l * l * mp + Ip * M * R * R * l * l * mw +
       Im * L * L * R * R * mp * mw + 2.0 * Im * L * Lm * M * R * R * mw + L * L * M * R * R * l * l * mp * mw);
  double a_2_5 =
      (Iw * L * M * M * g * l * l + Iw * Lm * M * M * g * l * l + L * M * M * R * R * g * l * l * mw +
       Lm * M * M * R * R * g * l * l * mp + Lm * M * M * R * R * g * l * l * mw) /
      (Im * Ip * Iw + Ip * Iw * M * l * l + Im * Iw * L * L * mp + Im * Ip * R * R * mp + Im * Ip * R * R * mw +
       Im * Iw * L * L * M + Im * Iw * Lm * Lm * M + Im * Ip * M * R * R + Im * L * L * M * R * R * mw +
       Im * Lm * Lm * M * R * R * mp + Im * Lm * Lm * M * R * R * mw + 2.0 * Im * Iw * L * Lm * M +
       Iw * L * L * M * l * l * mp + Ip * M * R * R * l * l * mp + Ip * M * R * R * l * l * mw +
       Im * L * L * R * R * mp * mw + 2.0 * Im * L * Lm * M * R * R * mw + L * L * M * R * R * l * l * mp * mw);
  double a_4_5 =
      (Ip * M * M * R * R * g * l * l - L * Lm * M * M * R * R * g * l * l * mp) /
      (Im * Ip * Iw + Ip * Iw * M * l * l + Im * Iw * L * L * mp + Im * Ip * R * R * mp + Im * Ip * R * R * mw +
       Im * Iw * L * L * M + Im * Iw * Lm * Lm * M + Im * Ip * M * R * R + Im * L * L * M * R * R * mw +
       Im * Lm * Lm * M * R * R * mp + Im * Lm * Lm * M * R * R * mw + 2.0 * Im * Iw * L * Lm * M +
       Iw * L * L * M * l * l * mp + Ip * M * R * R * l * l * mp + Ip * M * R * R * l * l * mw +
       Im * L * L * R * R * mp * mw + 2.0 * Im * L * Lm * M * R * R * mw + L * L * M * R * R * l * l * mp * mw);
  double a_6_5 =
      (Ip * Iw * M * g * l + Iw * L * L * M * M * g * l + Iw * Lm * Lm * M * M * g * l + Ip * M * M * R * R * g * l +
       L * L * M * M * R * R * g * l * mw + Lm * Lm * M * M * R * R * g * l * mp + Lm * Lm * M * M * R * R * g * l * mw +
       2.0 * Iw * L * Lm * M * M * g * l + Iw * L * L * M * g * l * mp + Ip * M * R * R * g * l * mp +
       Ip * M * R * R * g * l * mw + 2.0 * L * Lm * M * M * R * R * g * l * mw + L * L * M * R * R * g * l * mp * mw) /
      (Im * Ip * Iw + Ip * Iw * M * l * l + Im * Iw * L * L * mp + Im * Ip * R * R * mp + Im * Ip * R * R * mw +
       Im * Iw * L * L * M + Im * Iw * Lm * Lm * M + Im * Ip * M * R * R + Im * L * L * M * R * R * mw +
       Im * Lm * Lm * M * R * R * mp + Im * Lm * Lm * M * R * R * mw + 2.0 * Im * Iw * L * Lm * M +
       Iw * L * L * M * l * l * mp + Ip * M * R * R * l * l * mp + Ip * M * R * R * l * l * mw +
       Im * L * L * R * R * mp * mw + 2.0 * Im * L * Lm * M * R * R * mw + L * L * M * R * R * l * l * mp * mw);
  // clang-format off
  a <<0.   ,1.,0.,0.,0.   ,0.,
      a_2_1,0.,0.,0.,a_2_5,0.,
      0.   ,0.,0.,1.,0.   ,0.,
      a_4_1,0.,0.,0.,a_4_5,0.,
      0.   ,0.,0.,0.,0.   ,1.,
      a_6_1,0.,0.,0.,a_6_5,0.;
  // clang-format on
}

void BalanceController::generateB(double R, double L, double Lm, double l, double mw, double mp, double M, double Iw,
                                  double Ip, double Im, double g, Eigen::Matrix<double, STATE_DIM, CONTROL_DIM>& b)
{
  double b_2_1 =
      -(Im * Iw + Im * M * R * R + Iw * M * l * l + Im * R * R * mp + Im * R * R * mw + Im * L * M * R +
        Im * Lm * M * R + M * R * R * l * l * mp + M * R * R * l * l * mw + Im * L * R * mp + L * M * R * l * l * mp) /
      (Im * Ip * Iw + Ip * Iw * M * l * l + Im * Iw * L * L * mp + Im * Ip * R * R * mp + Im * Ip * R * R * mw +
       Im * Iw * L * L * M + Im * Iw * Lm * Lm * M + Im * Ip * M * R * R + Im * L * L * M * R * R * mw +
       Im * Lm * Lm * M * R * R * mp + Im * Lm * Lm * M * R * R * mw + 2.0 * Im * Iw * L * Lm * M +
       Iw * L * L * M * l * l * mp + Ip * M * R * R * l * l * mp + Ip * M * R * R * l * l * mw +
       Im * L * L * R * R * mp * mw + 2.0 * Im * L * Lm * M * R * R * mw + L * L * M * R * R * l * l * mp * mw);
  double b_4_1 =
      (Im * Ip * R + Ip * M * R * l * l + Im * L * R * R * mp + Im * L * L * R * mp + Im * L * M * R * R +
       Im * L * L * M * R + Im * Lm * M * R * R + Im * Lm * Lm * M * R + 2.0 * Im * L * Lm * M * R +
       L * M * R * R * l * l * mp + L * L * M * R * l * l * mp) /
      (Im * Ip * Iw + Ip * Iw * M * l * l + Im * Iw * L * L * mp + Im * Ip * R * R * mp + Im * Ip * R * R * mw +
       Im * Iw * L * L * M + Im * Iw * Lm * Lm * M + Im * Ip * M * R * R + Im * L * L * M * R * R * mw +
       Im * Lm * Lm * M * R * R * mp + Im * Lm * Lm * M * R * R * mw + 2.0 * Im * Iw * L * Lm * M +
       Iw * L * L * M * l * l * mp + Ip * M * R * R * l * l * mp + Ip * M * R * R * l * l * mw +
       Im * L * L * R * R * mp * mw + 2.0 * Im * L * Lm * M * R * R * mw + L * L * M * R * R * l * l * mp * mw);
  double b_6_1 =
      -(Iw * L * M * l + Iw * Lm * M * l - Ip * M * R * l + L * M * R * R * l * mw + Lm * M * R * R * l * mp +
        Lm * M * R * R * l * mw + L * Lm * M * R * l * mp) /
      (Im * Ip * Iw + Ip * Iw * M * l * l + Im * Iw * L * L * mp + Im * Ip * R * R * mp + Im * Ip * R * R * mw +
       Im * Iw * L * L * M + Im * Iw * Lm * Lm * M + Im * Ip * M * R * R + Im * L * L * M * R * R * mw +
       Im * Lm * Lm * M * R * R * mp + Im * Lm * Lm * M * R * R * mw + 2.0 * Im * Iw * L * Lm * M +
       Iw * L * L * M * l * l * mp + Ip * M * R * R * l * l * mp + Ip * M * R * R * l * l * mw +
       Im * L * L * R * R * mp * mw + 2.0 * Im * L * Lm * M * R * R * mw + L * L * M * R * R * l * l * mp * mw);
  double b_2_2 =
      (Im * Iw + Im * M * R * R + Iw * M * l * l + Im * R * R * mp + Im * R * R * mw + M * R * R * l * l * mp +
       M * R * R * l * l * mw + Iw * L * M * l + Iw * Lm * M * l + L * M * R * R * l * mw + Lm * M * R * R * l * mp +
       Lm * M * R * R * l * mw) /
      (Im * Ip * Iw + Ip * Iw * M * l * l + Im * Iw * L * L * mp + Im * Ip * R * R * mp + Im * Ip * R * R * mw +
       Im * Iw * L * L * M + Im * Iw * Lm * Lm * M + Im * Ip * M * R * R + Im * L * L * M * R * R * mw +
       Im * Lm * Lm * M * R * R * mp + Im * Lm * Lm * M * R * R * mw + 2.0 * Im * Iw * L * Lm * M +
       Iw * L * L * M * l * l * mp + Ip * M * R * R * l * l * mp + Ip * M * R * R * l * l * mw +
       Im * L * L * R * R * mp * mw + 2.0 * Im * L * Lm * M * R * R * mw + L * L * M * R * R * l * l * mp * mw);
  double b_4_2 =
      -(Im * L * R * R * mp - Ip * M * R * R * l + Im * L * M * R * R + Im * Lm * M * R * R +
        L * M * R * R * l * l * mp + L * Lm * M * R * R * l * mp) /
      (Im * Ip * Iw + Ip * Iw * M * l * l + Im * Iw * L * L * mp + Im * Ip * R * R * mp + Im * Ip * R * R * mw +
       Im * Iw * L * L * M + Im * Iw * Lm * Lm * M + Im * Ip * M * R * R + Im * L * L * M * R * R * mw +
       Im * Lm * Lm * M * R * R * mp + Im * Lm * Lm * M * R * R * mw + 2.0 * Im * Iw * L * Lm * M +
       Iw * L * L * M * l * l * mp + Ip * M * R * R * l * l * mp + Ip * M * R * R * l * l * mw +
       Im * L * L * R * R * mp * mw + 2.0 * Im * L * Lm * M * R * R * mw + L * L * M * R * R * l * l * mp * mw);
  double b_6_2 =
      (Ip * Iw + Iw * L * L * M + Iw * Lm * Lm * M + Ip * M * R * R + Iw * L * L * mp + Ip * R * R * mp +
       Ip * R * R * mw + L * L * M * R * R * mw + Lm * Lm * M * R * R * mp + Lm * Lm * M * R * R * mw +
       2.0 * Iw * L * Lm * M + L * L * R * R * mp * mw + Iw * L * M * l + Iw * Lm * M * l +
       2.0 * L * Lm * M * R * R * mw + L * M * R * R * l * mw + Lm * M * R * R * l * mp + Lm * M * R * R * l * mw) /
      (Im * Ip * Iw + Ip * Iw * M * l * l + Im * Iw * L * L * mp + Im * Ip * R * R * mp + Im * Ip * R * R * mw +
       Im * Iw * L * L * M + Im * Iw * Lm * Lm * M + Im * Ip * M * R * R + Im * L * L * M * R * R * mw +
       Im * Lm * Lm * M * R * R * mp + Im * Lm * Lm * M * R * R * mw + 2.0 * Im * Iw * L * Lm * M +
       Iw * L * L * M * l * l * mp + Ip * M * R * R * l * l * mp + Ip * M * R * R * l * l * mw +
       Im * L * L * R * R * mp * mw + 2.0 * Im * L * Lm * M * R * R * mw + L * L * M * R * R * l * l * mp * mw);
  // clang-format off
  b <<0.   ,0.   ,
      b_2_1,b_2_2,
      0.   ,0.   ,
      b_4_1,b_4_2,
      0.   ,0.   ,
      b_6_1,b_6_2;
  // clang-format on
}
}  // namespace rm_chassis_controllers
PLUGINLIB_EXPORT_CLASS(rm_chassis_controllers::BalanceController, controller_interface::ControllerBase)
