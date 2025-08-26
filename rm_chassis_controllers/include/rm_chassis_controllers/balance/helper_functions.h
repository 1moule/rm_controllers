//
// Created by guanlin on 25-8-26.
//

#pragma once

#include <Eigen/Dense>
#include <vector>
#include <cstddef>
#include <memory>
#include "rm_chassis_controllers/balance/gen_A.h"
#include "rm_chassis_controllers/balance/gen_B.h"

namespace rm_chassis_controllers
{
constexpr static const int STATE_DIM = 6;
constexpr static const int CONTROL_DIM = 2;

struct ModelParams
{
  double L_weight;   // Length weight to wheel axis
  double Lm_weight;  // Length weight to mass center
  double l;          // Leg rest length
  double m_w;        // Wheel mass
  double m_p;        // Leg mass
  double M;          // Body mass
  double i_w;        // Wheel inertia
  double i_p;        // Leg inertia
  double i_m;        // Body inertia
  double r;          // Wheel radius
  double g;          // Gravity acceleration
};

inline void generateAB(const std::unique_ptr<ModelParams>& model_params, Eigen::Matrix<double, STATE_DIM, STATE_DIM>& a,
                       Eigen::Matrix<double, STATE_DIM, CONTROL_DIM>& b, double leg_length)
{
  double A[36] = { 0. }, B[12]{ 0. };
  double L = leg_length * model_params->L_weight;
  double Lm = leg_length * model_params->Lm_weight;
  gen_A(model_params->i_m, model_params->i_p, model_params->i_w, L, Lm, model_params->M, model_params->r,
        model_params->g, model_params->l, model_params->m_p, model_params->m_w, A);
  gen_B(model_params->i_m, model_params->i_p, model_params->i_w, L, Lm, model_params->M, model_params->r,
        model_params->l, model_params->m_p, model_params->m_w, B);

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
}

inline void polyfit(const std::vector<Eigen::Matrix<double, 2, 6>>& Ks, const std::vector<double>& L0s,
                    Eigen::Matrix<double, 4, 12>& coeffs)
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

inline double calculateSupportForce(double F, double Tp, double leg_length, double acc_z,
                                    Eigen::Matrix<double, STATE_DIM, 1> x, Eigen::Matrix<double, CONTROL_DIM, 1> u,
                                    const std::unique_ptr<ModelParams>& model_params)
{
  Eigen::Matrix<double, STATE_DIM, STATE_DIM> a;
  Eigen::Matrix<double, STATE_DIM, CONTROL_DIM> b;
  generateAB(model_params, a, b, leg_length);

  double P = F * cos(x(0)) + Tp * sin(x(0)) / leg_length;
  double ddot_zM = acc_z - model_params->g;
  auto ddot_x = a * x + b * u;
  double ddot_theta = ddot_x(1);
  double ddot_zw = ddot_zM - leg_length * cos(x(0)) + 2 * leg_length * x(1) * sin(x(0)) +
                   +leg_length * (ddot_theta * sin(x(0)) + x(1) * x(1) * cos(x(0)));
  double Fn = model_params->m_w * ddot_zw + model_params->m_w * model_params->g + P;
  return Fn;
}

}  // namespace rm_chassis_controllers
