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
  std::string left_wheel_joint, right_wheel_joint;
  if (!controller_nh.getParam("left/wheel_joint", left_wheel_joint) ||
      !controller_nh.getParam("right/wheel_joint", right_wheel_joint))
  {
    ROS_ERROR("Some Joints' name doesn't given. (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  left_wheel_joint_handle_ = robot_hw->get<hardware_interface::EffortJointInterface>()->getHandle(left_wheel_joint);
  right_wheel_joint_handle_ = robot_hw->get<hardware_interface::EffortJointInterface>()->getHandle(right_wheel_joint);
  joint_handles_.push_back(left_wheel_joint_handle_);
  joint_handles_.push_back(right_wheel_joint_handle_);

  // m_w is mass of single wheel
  // m is mass of the robot except wheels and momentum_blocks
  // i_w is the moment of inertia of the wheel around the rotational axis of the motor
  // i_m is the moment of inertia of the robot around the y-axis of base_link coordinate.
  // l is the vertical component of the distance between the wheel center and the center of mass of robot
  double m_w, m, i_w, i_m, l, g;

  if (!controller_nh.getParam("m_w", m_w))
  {
    ROS_ERROR("Params m_w doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("m", m))
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
  if (!controller_nh.getParam("l", l))
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
  if (!controller_nh.getParam("wheel_base", wheel_base_))
  {
    ROS_ERROR("Params wheel_base_ doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  controller_nh.getParam("position_offset", position_offset_);
  controller_nh.getParam("position_clear_threshold", position_clear_threshold_);

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
  double D = (m + m_w) * (i_m + m * l * l) - m * m * l * l;
  double a = m * g * l * (m + m_w) / D;
  double b = -m * l / (D * wheel_radius_);
  double c = -m * m * l * l * g / D;
  double d = (1 / (m + m_w) * wheel_radius_) + (m * m * l * l / ((m + m_w) * D * wheel_radius_));
  // clang-format off
  a_<<0.,1.,0.,0.,
      a ,0.,0.,0.,
      0.,0.,0.,1.,
      c ,0.,0.,0.;
  b_<<0.,
      b ,
      0.,
      d;
  // clang-format on

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
  pid_yaw_vel_.computeCommand(0.0 - angular_vel_base_.z, period);
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
}  // namespace rm_chassis_controllers
PLUGINLIB_EXPORT_CLASS(rm_chassis_controllers::BalanceController, controller_interface::ControllerBase)
