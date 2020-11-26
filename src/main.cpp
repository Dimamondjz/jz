#include "mpcc_planner/mpcc_planner.hpp"
int main(int argc, char** argv) {
  mpcc::MpccPlanner mpccPlanner;
  mpccPlanner.init();
  kinematic_model::VectorX x0;
  x0 << 0, 0, 0, 0, 0;
  std::vector<double> control = mpccPlanner.solveQP(x0);  //速度 加速度 舵轮转角
  for (auto c : control) {
    std::cout << c << std::endl;
  }
}
