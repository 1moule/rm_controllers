//
// Created by qiayuan on 8/14/20.
//

#ifndef SRC_RM_COMMON_INCLUDE_BULLET_SOLVER_H_
#define SRC_RM_COMMON_INCLUDE_BULLET_SOLVER_H_

#include <rm_msgs/GimbalTrackCmd.h>
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_buffer.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>
#include "cpp_types.h"
template<typename T>
class BulletSolver {
 public:
  BulletSolver(T resistance_coff, T g, T delay, T dt, T timeout) :
      resistance_coff_(resistance_coff),
      g_(g), delay_(delay), dt_(dt),
      timeout_(timeout) {
    ros::NodeHandle n("~/");
    path_pub_ = n.advertise<visualization_msgs::Marker>("bullet_model", 200);
  };
  virtual ~BulletSolver() = default;
  virtual void setTarget(const DVec<T> &pos, const DVec<T> &vel) = 0;
  virtual void setBulletSpeed(T speed) { bullet_speed_ = speed; };
  virtual void solve(const DVec<T> &angle_init) = 0;
  virtual void output(Vec2<T> &angle_solved) = 0;
  virtual geometry_msgs::TransformStamped run(const ros::Time &time,
                                              realtime_tools::RealtimeBuffer<rm_msgs::GimbalTrackCmd> cmd_track_rt_buffer) = 0;
  virtual void modelRviz(double x_deviation, double y_deviation, double z_deviation) = 0;
  virtual std::vector<Vec3<T>> getPointData3D() = 0;
 protected:
  T bullet_speed_{};
  T resistance_coff_, g_, delay_, dt_, timeout_;
  ros::Publisher path_pub_;
};

template<typename T>
class Bullet2DSolver : public BulletSolver<T> {
 public:
  using BulletSolver<T>::BulletSolver;
  void setTarget(const DVec<T> &pos, const DVec<T> &vel) override {
    target_x_ = pos[0] + vel[0] * this->delay_;
    target_z_ = pos[1] + vel[1] * this->delay_;
    target_dx_ = vel[0];
    target_dz_ = vel[1];
  };
  void solve(const DVec<T> &angle_init) override;
  void output(Vec2<T> &angle_solved) override {
    angle_solved[0] = pitch_solved_;
  }
 protected:
  virtual T computeError(T pitch) = 0;
  T target_x_{}, target_z_{}, target_dx_{}, target_dz_{};
  T fly_time_{};
  T pitch_solved_;
};

template<typename T>
class Iter2DSolver : public Bullet2DSolver<T> {
 public:
  using Bullet2DSolver<T>::Bullet2DSolver;
  using Bullet2DSolver<T>::solve;
 private:
  T computeError(T pitch) override;
};

template<typename T>
class Approx2DSolver : public Bullet2DSolver<T> {
 public:
  using Bullet2DSolver<T>::Bullet2DSolver;
  using Bullet2DSolver<T>::solve;
 private:
  T computeError(T pitch) override;
};

template<typename T>
class Bullet3DSolver : public BulletSolver<T> {
 public:
  using BulletSolver<T>::BulletSolver;
  void setTarget(const DVec<T> &pos, const DVec<T> &vel) override {
    target_x_ = pos[0] + vel[0] * this->delay_;
    target_y_ = pos[1] + vel[1] * this->delay_;
    target_z_ = pos[2] + vel[2] * this->delay_;
    target_dx_ = vel[0];
    target_dy_ = vel[1];
    target_dz_ = vel[2];
  };
  void solve(const DVec<T> &angle_init) override;
  geometry_msgs::TransformStamped run(const ros::Time &time,
                                      realtime_tools::RealtimeBuffer<rm_msgs::GimbalTrackCmd> cmd_track_rt_buffer) override;
  void modelRviz(double x_deviation, double y_deviation, double z_deviation) override;
  void output(Vec2<T> &angle_solved) override {
    angle_solved[0] = yaw_solved_;
    angle_solved[1] = pitch_solved_;
  }
  std::vector<Vec3<T>> getPointData3D() override;
 protected:
  virtual double computeError(T yaw, T pitch, T *error_polar) = 0;
  T target_x_{}, target_y_{}, target_z_{},
      target_dx_{}, target_dy_{}, target_dz_{};
  T fly_time_{};
  T pitch_solved_, yaw_solved_;
  Vec2<T> angle_init_{};
  Vec2<T> angle_solved_{};
  Vec3<T> pos_{};
  Vec3<T> vel_{};
  std::vector<Vec3<T>> model_data_;
};

template<typename T>
class Iter3DSolver : public Bullet3DSolver<T> {
 public:
  using Bullet3DSolver<T>::Bullet3DSolver;
  using Bullet3DSolver<T>::solve;
  using Bullet3DSolver<T>::getPointData3D;
 private:
  double computeError(T yaw, T pitch, T *error_polar) override;
};

template<typename T>
class Approx3DSolver : public Bullet3DSolver<T> {
 public:
  using Bullet3DSolver<T>::Bullet3DSolver;
  using Bullet3DSolver<T>::solve;
  using Bullet3DSolver<T>::getPointData3D;
 private:
  double computeError(T yaw, T pitch, T *error_polar) override;
};
#endif //SRC_RM_COMMON_INCLUDE_BULLET_SOLVER_H_
