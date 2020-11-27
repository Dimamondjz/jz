//
// Created by clint on 2020/11/25.
//

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

KinematicModel::KinematicModel() {
  YAML::Node config = YAML::LoadFile("../config/parameters.yaml");
  ll_ = config["ll"].as<double>();
  dt_ = config["dt"].as<double>();
}

MatrixAd KinematicModel::Ad(const VectorX &state, const VectorU &input) {
  double phi = state.coeff(2);
  double v = state.coeff(3);
  double delta = input.coeff(1);
  MatrixAd Ad;
  using namespace std;
  Ad << 1, 0, -v * sin(phi) * dt_, cos(phi) * dt_, 0, 0, 1, v * cos(phi) * dt_,
      sin(phi) * dt_, 0, 0, 0, 1, tan(delta) / ll_ * dt_, 0, 0, 0, 0, 1, 0, 0,
      0, 0, 0, 1;
  return Ad;
}

MatrixBd KinematicModel::Bd(const VectorX &state, const VectorU &input) {
  double phi = state.coeff(2);
  double v = state.coeff(3);
  double delta = input.coeff(1);
  MatrixBd Bd;
  using namespace std;
  Bd << 0, 0, 0, 0, 0, 0, 0, v / (ll_ * cos(delta) * cos(delta)) * dt_, 0, dt_,
      0, 0, 0, 0, dt_;
  return Bd;
}

Matrixgd KinematicModel::gd(const VectorX &state, const VectorU &input) {
  double phi = state.coeff(2);
  double v = state.coeff(3);
  double delta = input.coeff(1);
  Matrixgd gd;
  using namespace std;
  gd << v * cos(phi) * dt_ + v * sin(phi) * dt_ * phi - cos(phi) * dt_ * v,
      v * sin(phi) * dt_ - v * cos(phi) * dt_ * phi - sin(phi) * dt_ * v,
      v / ll_ * tan(delta) * dt_ - tan(delta) / ll_ * dt_ * v -
          v / (ll_ * cos(delta) * cos(delta)) * dt_ * delta,
      0, 0;
  return gd;
}

VectorX KinematicModel::nextState(const VectorX &state, const VectorU &input) {
  double x = state.coeff(0);
  double y = state.coeff(1);
  double phi = state.coeff(2);
  double v = state.coeff(3);
  double s = state.coeff(4);
  double a = input.coeff(0);
  double delta = input.coeff(1);
  double vs = input.coeff(2);
  using namespace std;
  double x_k = x + v * cos(phi) * dt_;
  double y_k = y + v * sin(phi) * dt_;
  double phi_k = phi + v / ll_ * 1.05 * tan(delta) * dt_;
  double v_k = v + a * dt_;
  double s_k = s + vs * dt_;
  VectorX nextState;
  nextState << x_k, y_k, phi_k, v_k, s_k;
  return nextState;
}

}  // namespace kinematic_model