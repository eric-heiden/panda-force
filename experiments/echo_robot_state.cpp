#include <iostream>
#include <chrono>
#include <thread>

#include <fstream>

#include <Eigen/Dense>

#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/model.h>

#include "state_recorder.hpp"

template <typename Scalar, std::size_t Size>
void print(const std::array<Scalar, Size> &a)
{
  for (size_t i = 0; i < Size; ++i)
  {
    std::cout << a[i];
    if (i < Size - 1)
    {
      std::cout << ", ";
    }
  }
  std::cout << std::endl;
}

int main(int argc, char **argv)
{
  using namespace std::chrono_literals;

  // if (argc != 2) {
  //   std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
  //   return -1;
  // }

  try
  {
    // franka::Robot robot(argv[1]);
    std::string franka_address = "172.16.0.2";
    franka::Robot robot(franka_address);

    // load the kinematics and dynamics model
    franka::Model model = robot.loadModel();

    

    // franka::RobotState initial_state = robot.readOnce();
    // std::array<double, 7> init_gravity_array = model.gravity(initial_state);

    StateRecorder state_recorder("log_panda_nop.json", model);
    state_recorder.print_to_stdout = true;

    for (int i = 0; i < 200; ++i)
    {
      robot.read(state_recorder);
      std::this_thread::sleep_for(50ms);
    }

    state_recorder.save();

    std::cout << "Done." << std::endl;
  }
  catch (franka::Exception const &e)
  {
    std::cerr << "ERROR: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
