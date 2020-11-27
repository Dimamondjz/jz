cmake_minimum_required(VERSION 3.0.2)
project(demo)

add_compile_options(-std=c++11 -O3)
find_package(Eigen3)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
include_directories(
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
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY  # 指定库文件生成目录
    ${CMAKE_CURRENT_SOURCE_DIR}/lib) 
add_library(osqp_interface SHARED  # 生成的osqp_interface动态连接库文件
    ${CMAKE_CURRENT_SOURCE_DIR}/src/osqp_interface.cpp) 
add_library(mpcc_planner SHARED  # 生成的mpcc_planner动态连接库文件
    ${CMAKE_CURRENT_SOURCE_DIR}/src/mpcc_planner.cpp) 
add_library(arc_spline SHARED  # 生成的arc_spline动态连接库文件
    ${CMAKE_CURRENT_SOURCE_DIR}/src/arc_spline.cpp) 
add_library(kinematic_model SHARED  # 生成的kinematic_model动态连接库文件
    ${CMAKE_CURRENT_SOURCE_DIR}/src/kinematic_model.cpp) 
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
add_executable(${PROJECT_NAME} 
    src/demo.cpp
    )

target_link_libraries(${PROJECT_NAME} 
    osqp_interface
    osqp
    mpcc_planner
    arc_spline
    kinematic_model
    yaml-cpp
)
