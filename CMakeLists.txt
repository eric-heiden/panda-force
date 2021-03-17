cmake_minimum_required(VERSION 3.4)
project(panda_force)

# SET(BUILD_TESTS OFF CACHE BOOL "Build tests")
# SET(BUILD_EXAMPLES OFF CACHE BOOL "Build example code")
# SET(BUILD_DOCUMENTATION OFF CACHE BOOL "Build documentation")
# add_subdirectory(libfranka)

# find_dependency(Poco REQUIRED COMPONENTS Net Foundation)
# find_dependency(Eigen3 REQUIRED)

# set(THREADS_PREFER_PTHREAD_FLAG ON)
# find_dependency(Threads REQUIRED)

find_package(Franka REQUIRED)


add_executable(echo_robot_state experiments/echo_robot_state.cpp)
target_link_libraries(echo_robot_state Franka::Franka)