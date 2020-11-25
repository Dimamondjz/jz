cmake_minimum_required(VERSION 3.0.2)
project(mpcc_planner_test)

add_compile_options(-std=c++11 -O3)


find_package(Eigen3)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})

include_directories(
  osqp_interface/include/
  include
  /usr/local/include/osqp
  /usr/local/include/yaml-cpp
)
link_directories( # 库文件搜索路径
    /usr/local/lib/
    )
add_executable(mpcc_planner_test
  src/mpcc_planner_node.cpp
  osqp_interface/include/osqp_interface/osqp_interface.h
  osqp_interface/src/osqp_interface.cpp
)

target_link_libraries(mpcc_planner_test
    yaml-cpp
        osqp
)
