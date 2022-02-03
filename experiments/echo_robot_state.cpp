#include <chrono>
#include <iostream>
#include <thread>

#include <fstream>

#include <functional>

#include <Eigen/Dense>

#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

#include "state_recorder.hpp"

#include "settings.h"

template <typename Scalar, std::size_t Size>
void print(const std::array<Scalar, Size> &a) {
  for (size_t i = 0; i < Size; ++i) {
    std::cout << a[i];
    if (i < Size - 1) {
      std::cout << ", ";
    }
  }
  std::cout << std::endl;
}

int main(int argc, char **argv) {
  using namespace std::chrono_literals;

  std::cout << "PANDA_HOME: " << PANDA_HOME << std::endl;

  // if (argc != 2) {
  //   std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
  //   return -1;
  // }

  try {
    // franka::Robot robot(argv[1]);
    std::string franka_address = "172.16.0.2";
    franka::Robot robot(franka_address);
    settings::set_EE_properties(
        robot, std::string(PANDA_HOME) +
                   std::string("/edc-knife-endeffector-config.json"));

    // load the kinematics and dynamics model
    franka::Model model = robot.loadModel();

    // franka::RobotState initial_state = robot.readOnce();
    // std::array<double, 7> init_gravity_array = model.gravity(initial_state);

    StateRecorder state_recorder("log_panda_nop.json", model);
    state_recorder.print_to_stdout = true;

    // auto recording_fn = std::bind(&StateRecorder::operator(), state_recorder,
    // std::placeholders::_1);

    for (int i = 0; i < 400; ++i) {
      // robot.read(recording_fn);
      robot.read([&state_recorder](const franka::RobotState &robot_state) {
        state_recorder(robot_state);
        return false;
      });
      std::this_thread::sleep_for(20ms);
    }

    state_recorder.save();

    std::cout << "Done." << std::endl;
  } catch (franka::Exception const &e) {
    std::cerr << "ERROR: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
