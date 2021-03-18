#include <iostream>
#include <chrono>
#include <thread>

#include <fstream>

#include <franka/exception.h>
#include <franka/robot.h>

#include "json.hpp"

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
  typedef std::chrono::high_resolution_clock Time;
  typedef std::chrono::milliseconds ms;
  auto t0 = Time::now();

  // if (argc != 2) {
  //   std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
  //   return -1;
  // }

  try
  {
    // franka::Robot robot(argv[1]);
    std::string franka_address = "172.16.0.2";
    franka::Robot robot(franka_address);

    nlohmann::json log;
    log["time"] = {};
    log["tau_ext_hat_filtered"] = {};
    log["K_F_ext_hat_K"] = {};
    log["O_F_ext_hat_K"] = {};
    log["robot_time"] = {};
    log["F_T_EE"] = {};
    log["F_T_NE"] = {};
    log["F_x_Cload"] = {};
    log["F_x_Ctotal"] = {};
    log["I_ee"] = {};
    log["I_load"] = {};
    log["I_total"] = {};

    for (int i = 0; i < 200; ++i)
    {
      robot.read([&t0, &log](const franka::RobotState &robot_state) {
        // Printing to std::cout adds a delay. This is acceptable for a read loop such as this, but
        // should not be done in a control loop.
        // std::cout << robot_state << std::endl;

        auto ti = Time::now();
        std::chrono::duration<double> fs = ti - t0;
        auto d = std::chrono::duration_cast<std::chrono::microseconds>(fs);
        double time = d.count() / 1e6;
        std::cout << "time: " << time << std::endl;

        log["time"].push_back(time);
        log["robot_time"].push_back(robot_state.time.toMSec());
        log["F_T_EE"].push_back(robot_state.F_T_EE);
        log["F_T_NE"].push_back(robot_state.F_T_NE);
        log["F_x_Cload"].push_back(robot_state.F_x_Cload);
        log["F_x_Ctotal"].push_back(robot_state.F_x_Ctotal);
        log["I_ee"].push_back(robot_state.I_ee);
        log["I_load"].push_back(robot_state.I_load);
        log["I_total"].push_back(robot_state.I_total);
        log["tau_ext_hat_filtered"].push_back(robot_state.tau_ext_hat_filtered);
        log["K_F_ext_hat_K"].push_back(robot_state.K_F_ext_hat_K);
        log["O_F_ext_hat_K"].push_back(robot_state.O_F_ext_hat_K);

        std::cout << "tau_ext_hat_filtered: ";
        print(robot_state.tau_ext_hat_filtered);
        std::cout << "K_F_ext_hat_K: ";
        print(robot_state.K_F_ext_hat_K);
        std::cout << "O_F_ext_hat_K: ";
        print(robot_state.O_F_ext_hat_K);
        std::cout << std::endl;

        return false; // do not repeat here
      });
      std::this_thread::sleep_for(50ms);
    }

    std::ofstream file("log_panda_nop.json");
    file << log;

    std::cout << "Done." << std::endl;
  }
  catch (franka::Exception const &e)
  {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
