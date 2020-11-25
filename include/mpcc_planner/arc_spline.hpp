#ifndef ARC_SPLINE_HPP
#define ARC_SPLINE_HPP

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

}  // namespace arc_spline

#endif
