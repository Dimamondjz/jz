#ifndef KINEMATIC_MODEL_HPP
#define KINEMATIC_MODEL_HPP

#include <Eigen/Core>
#include <iostream>

#include "yaml.h"
/*
 *   x(k+1) =   x(k) + v(k) * cos(phi(k)) * dt
 *   y(k+1) =   y(k) + v(k) * sin(phi(k)) * dt
 * phi(k+1) = phi(k) + v(k) / ll * tan(delta(k)) * dt
 *   v(k+1) =   v(k) + a(k) * dt
 *   s(k+1) =   s(k) + vs(k) *dt_
 */

namespace kinematic_model {

const int n = 5;  // state x y phi v s                           //小车状态
                  // 全局坐标x,y,惯性航向,速度,里程
const int m = 3;  // input a beta vs                             //控制输入
                  // 加速度,速度转向角度,里程速度
typedef Eigen::Matrix<double, n, n> MatrixAd;
typedef Eigen::Matrix<double, n, m> MatrixBd;
typedef Eigen::Matrix<double, n, 1> Matrixgd;
typedef Eigen::Matrix<double, n, 1> VectorX;
typedef Eigen::Matrix<double, m, 1> VectorU;

class KinematicModel {
 private:
  double ll_;
  double dt_;

 public:
  KinematicModel();
  ~KinematicModel(){};
  MatrixAd Ad(const VectorX &state, const VectorU &input);
  MatrixBd Bd(const VectorX &state, const VectorU &input);
  Matrixgd gd(const VectorX &state, const VectorU &input);
  VectorX nextState(const VectorX &state, const VectorU &input);
  typedef std::shared_ptr<KinematicModel> Ptr;
};

}  // namespace kinematic_model

#endif
