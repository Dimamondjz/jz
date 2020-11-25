//
// Created by clint on 2020/11/25.
//

#include <Eigen/Core>

#include "tk_spline/spline.hpp"

namespace arc_spline {
class ArcSpline {
 private:
  double arcL_;
  tk::spline xs_, ys_;
  std::vector<double> sL_;
  std::vector<double> xL_;
  std::vector<double> yL_;

 public:
  ArcSpline(){};
  ~ArcSpline(){};
  void setWayPoints(const std::vector<double> &x_waypoints,
                    const std::vector<double> &y_waypoints);
  Eigen::Vector2d operator()(double s, int n = 0);
  double findS(Eigen::Vector2d p);
  inline double arcL() { return arcL_; };
  void set_arcL(double s) { arcL_ = s; }
};

void ArcSpline::setWayPoints(const std::vector<double> &x_waypoints,
                             const std::vector<double> &y_waypoints) {
  assert(x_waypoints.size() == y_waypoints.size());
  tk::spline xt_, yt_;
  std::vector<double> t_list;
  std::vector<double> x_list = x_waypoints;
  std::vector<double> y_list = y_waypoints;

  // add front to back -> a loop
  // x_list.push_back(x_list.front());
  // y_list.push_back(y_list.front());
  for (int i = 0; i < x_list.size(); ++i) {
    t_list.push_back(i);
  }
  xt_.set_points(t_list, x_list);
  yt_.set_points(t_list, y_list);
  double res = 1e-3;
  sL_.resize(0);
  sL_.push_back(0);
  xL_.push_back(xt_(0));
  yL_.push_back(yt_(0));
  // calculate arc length approximately
  double last_s = 0;
  for (double t = res; t < t_list.back(); t += res) {
    double left_arc = res * std::sqrt(xt_(t - res, 1) * xt_(t - res, 1) +
                                      yt_(t - res, 1) * yt_(t - res, 1));
    double right_arc =
        res * std::sqrt(xt_(t, 1) * xt_(t, 1) + yt_(t, 1) * yt_(t, 1));
    sL_.push_back(last_s + (left_arc + right_arc) / 2);
    last_s += (left_arc + right_arc) / 2;
    xL_.push_back(xt_(t));
    yL_.push_back(yt_(t));
  }
  xs_.set_points(sL_, xL_);
  ys_.set_points(sL_, yL_);
  arcL_ = sL_.back();
};

Eigen::Vector2d ArcSpline::operator()(double s, int n /* =0 */) {
  assert(n >= 0 && n <= 2);
  Eigen::Vector2d p(xs_(s, n), ys_(s, n));
  return p;
};

double ArcSpline::findS(Eigen::Vector2d p) {
  // TODO
}

}  // namespace arc_spline