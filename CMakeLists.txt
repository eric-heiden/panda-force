cmake_minimum_required(VERSION 3.4)
project(panda_force)

find_package(Franka REQUIRED) 
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

add_executable(echo_robot_state experiments/echo_robot_state.cpp)
target_link_libraries(echo_robot_state Franka::Franka Eigen3::Eigen)

add_executable(force_control experiments/force_control.cpp libfranka/examples/examples_common.cpp)
target_include_directories(force_control PRIVATE libfranka/examples)
target_link_libraries(force_control Franka::Franka Eigen3::Eigen)