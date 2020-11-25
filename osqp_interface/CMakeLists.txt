cmake_minimum_required(VERSION 3.0.2)
project(osqp_interface)

add_compile_options(-std=c++11 -O3)

find_package(Eigen3)
find_package(osqp REQUIRED)

include_directories(
    include
    ${EIGEN3_INCLUDE_DIR}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}.cpp
)

target_link_libraries(${PROJECT_NAME} 
    osqp::osqp
)

