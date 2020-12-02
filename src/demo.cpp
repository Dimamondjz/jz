/*
Demo
变量说明
x0:状态输入(x,y,航向角,速度,里程)
control:控制输出(速度 加速度 舵轮转角)
 */
#include <unistd.h>

#include <chrono>

#include "mpcc_planner/mpcc_planner.hpp"
int main(int argc, char** argv) {
  // close(STDOUT_FILENO);
  mpcc::MpccPlanner mpccPlanner;
  mpccPlanner.init();
  kinematic_model::VectorX x0;
  x0 << 0, 0, 0, 0, 0;  // 状态输入
  // Demo
  std::vector<double> control = mpccPlanner.solveQP(x0);
  for (int i = 0; i < 3; i++) {
    std::cout << control[i] << std::endl;
  }

  /* // 仿真
  double i = 0, sum = 0;
  while (true) {
    auto start = std::chrono::high_resolution_clock::now();
    std::vector<double> control = mpccPlanner.solveQP(x0);
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> tm = end - start;  // 毫秒
    ++i;
    sum += tm.count();
    std::cout << "average dtime: " << sum / i << "ms" << std::endl;
    for (int i = 3; i < control.size(); i++) {
      x0.coeffRef(i - 3) = control[i];
    }
  } */
  return 0;
}