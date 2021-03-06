#include <cmath>
#include <iostream>
#include <atomic>
#include <mutex>
#include <thread>

#include <franka/exception.h>
#include <franka/robot.h>

#include "examples_common.h"
#include "state_recorder.hpp"

/**
 * @example generate_cartesian_velocity_motion.cpp
 * An example showing how to generate a Cartesian velocity motion.
 *
 * @warning Before executing this example, make sure there is enough space in front of the robot.
 */

int main(int argc, char **argv)
{
  //   if (argc != 2) {
  //     std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
  //     return -1;
  // }
  std::string franka_address = "172.16.0.2";
  franka::Robot robot(franka_address);
  setDefaultBehavior(robot);

  franka::Model model = robot.loadModel();

  StateRecorder state_recorder("log_panda_cartesian_velocity.json", model);

  const double print_rate = 10.0;
  std::atomic_bool running{true};

  struct
  {
    std::mutex mutex;
    bool has_data;
    std::array<double, 7> tau_d_last;
    franka::RobotState robot_state;
    std::array<double, 7> gravity;
  } print_data{};

  std::thread print_thread([print_rate, &print_data, &running, &state_recorder]() {
    while (running)
    {
      // Sleep to achieve the desired print rate.
      std::this_thread::sleep_for(
          std::chrono::milliseconds(static_cast<int>((1.0 / print_rate * 1000.0))));

      // Try to lock data to avoid read write collisions.
      if (print_data.mutex.try_lock())
      {
        if (print_data.has_data)
        {
          state_recorder(print_data.robot_state);
          print_data.has_data = false;
        }
        print_data.mutex.unlock();
      }
    }
    state_recorder.save();
  });

  try
  {

    // First move the robot to a suitable joint configuration
    std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    MotionGenerator motion_generator(0.5, q_goal);
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(motion_generator);
    std::cout << "Finished moving to initial joint configuration." << std::endl;

    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set the joint impedance.
    robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});

    // Set the collision behavior.
    std::array<double, 7> lower_torque_thresholds_nominal{
        {25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.}};
    std::array<double, 7> upper_torque_thresholds_nominal{
        {35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0}};
    std::array<double, 7> lower_torque_thresholds_acceleration{
        {25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.0}};
    std::array<double, 7> upper_torque_thresholds_acceleration{
        {35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0}};
    std::array<double, 6> lower_force_thresholds_nominal{{30.0, 30.0, 30.0, 25.0, 25.0, 25.0}};
    std::array<double, 6> upper_force_thresholds_nominal{{40.0, 40.0, 40.0, 35.0, 35.0, 35.0}};
    std::array<double, 6> lower_force_thresholds_acceleration{{30.0, 30.0, 30.0, 25.0, 25.0, 25.0}};
    std::array<double, 6> upper_force_thresholds_acceleration{{40.0, 40.0, 40.0, 35.0, 35.0, 35.0}};
    robot.setCollisionBehavior(
        lower_torque_thresholds_acceleration, upper_torque_thresholds_acceleration,
        lower_torque_thresholds_nominal, upper_torque_thresholds_nominal,
        lower_force_thresholds_acceleration, upper_force_thresholds_acceleration,
        lower_force_thresholds_nominal, upper_force_thresholds_nominal);

    double time_max = 4.0;
    double v_max = 0.1;
    double angle = M_PI / 4.0;
    double time = 0.0;
    robot.control([=, &time, &print_data, &running](const franka::RobotState &state,
                                                    franka::Duration period) -> franka::CartesianVelocities {
      time += period.toSec();

      if (print_data.mutex.try_lock())
      {
        print_data.has_data = true;
        print_data.robot_state = state;
        // print_data.tau_d_last = tau_d_rate_limited;
        // print_data.gravity = model.gravity(state);
        print_data.mutex.unlock();
      }

      double cycle = std::floor(pow(-1.0, (time - std::fmod(time, time_max)) / time_max));
      double v = cycle * v_max / 2.0 * (1.0 - std::cos(2.0 * M_PI / time_max * time));
      double v_x = std::cos(angle) * v;
      double v_z = -std::sin(angle) * v;
      std::cout << "v_z:  " << v_z << "\n";

      franka::CartesianVelocities output = {{0.0, 0.0, v_z, 0.0, 0.0, 0.0}};
      if (time >= 2 * time_max)
      {
        std::cout << std::endl
                  << "Finished motion, shutting down example" << std::endl;
        running = false;
        return franka::MotionFinished(output);
      }
      return output;
    });
  }
  catch (const franka::Exception &e)
  {
    running = false;
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
  }
  if (print_thread.joinable())
  {
    print_thread.join();
  }

  return EXIT_SUCCESS;
}
