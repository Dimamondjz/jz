#include "mpcc_planner/mpcc_planner.hpp"
int main(int argc, char** argv) {
  mpcc::MpccPlanner mpccPlanner;
  mpccPlanner.init();
  kinematic_model::VectorX x0;
  x0 << 0, 0, 0, 0, 15;
  mpccPlanner.solveQP(x0);
}
