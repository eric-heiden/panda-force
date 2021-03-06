cmake_minimum_required(VERSION 3.4)
project(panda_force)

set(CMAKE_CXX_STANDARD 17)

add_definitions(-DPANDA_HOME="${CMAKE_CURRENT_LIST_DIR}")

find_package(Franka REQUIRED) 
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_executable(echo_robot_state experiments/echo_robot_state.cpp)
target_link_libraries(echo_robot_state Franka::Franka Eigen3::Eigen)

add_executable(force_control experiments/force_control.cpp libfranka/examples/examples_common.cpp)
target_include_directories(force_control PRIVATE libfranka/examples)
target_link_libraries(force_control Franka::Franka Eigen3::Eigen Threads::Threads)

add_executable(cartesian_velocity_control experiments/cartesian_velocity_control.cpp libfranka/examples/examples_common.cpp)
target_include_directories(cartesian_velocity_control PRIVATE libfranka/examples)
target_link_libraries(cartesian_velocity_control Franka::Franka Eigen3::Eigen Threads::Threads)

add_executable(cut_vertical experiments/cut_vertical.cpp libfranka/examples/examples_common.cpp)
target_include_directories(cut_vertical PRIVATE libfranka/examples)
target_link_libraries(cut_vertical Franka::Franka Eigen3::Eigen Threads::Threads)

add_executable(cut_vertical_edc experiments/cut_vertical_edc.cpp libfranka/examples/examples_common.cpp)
target_include_directories(cut_vertical_edc PRIVATE libfranka/examples)
target_link_libraries(cut_vertical_edc Franka::Franka Eigen3::Eigen Threads::Threads)

find_package(cppzmq)

if (cppzmq_FOUND)
  add_executable(zmq_control_server experiments/zmq_control_server.cpp libfranka/examples/examples_common.cpp)
  target_compile_definitions(zmq_control_server PUBLIC -DUSE_ZMQ=1)
  target_link_libraries(zmq_control_server PUBLIC ${cppzmq_LIBRARIES} zmq Franka::Franka Eigen3::Eigen Threads::Threads)
  target_include_directories(zmq_control_server PUBLIC ${cppzmq_INCLUDE_DIRS} libfranka/examples)
else()
  target_compile_definitions(zmq_control_server PUBLIC -DUSE_ZMQ=0)
endif ()
