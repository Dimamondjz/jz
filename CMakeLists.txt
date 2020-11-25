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
        include/mpcc_planner
)
link_directories( # 库文件搜索路径
    /usr/local/lib/
        lib/
    )
#
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib) # 指定库文件生成目录
add_library(mpcc_planner SHARED ${CMAKE_CURRENT_SOURCE_DIR}/src/mpcc_planner.cpp) # 生成的库文件
add_library(arc_spline SHARED ${CMAKE_CURRENT_SOURCE_DIR}/src/arc_spline.cpp) # 生成的库文件
add_library(kinematic_model SHARED ${CMAKE_CURRENT_SOURCE_DIR}/src/kinematic_model.cpp) # 生成的库文件
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
add_executable(mpcc_planner_test
        src/main.cpp
        osqp_interface/include/osqp_interface/osqp_interface.h
        osqp_interface/src/osqp_interface.cpp
        )

target_link_libraries(mpcc_planner_test
        osqp
        mpcc_planner
        arc_spline
        kinematic_model
            yaml-cpp
)
