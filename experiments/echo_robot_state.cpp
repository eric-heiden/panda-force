#include <iostream>
#include <chrono>
#include <thread>

#include <fstream>

#include <Eigen/Dense>

#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/model.h>

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

    // load the kinematics and dynamics model
    franka::Model model = robot.loadModel();

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
    log["EE_T_K"] = {};
    log["O_T_EE"] = {};
    log["wrench"] = {};
    log["force"] = {};
    log["q"] = {};
    log["qd"] = {};
    log["tau"] = {};
    log["jacobian"] = {};
    log["coriolis"] = {};
    log["gravity"] = {};
    log["EE_position"] = {};

    franka::RobotState initial_state = robot.readOnce();
    std::array<double, 7> init_gravity_array = model.gravity(initial_state);

    for (int i = 0; i < 200; ++i)
    {
      robot.read([&t0, &log, &model](const franka::RobotState &robot_state) {
        // Printing to std::cout adds a delay. This is acceptable for a read loop such as this, but
        // should not be done in a control loop.
        // std::cout << robot_state << std::endl;

        std::array<double, 7> gravity_array = model.gravity(robot_state);
        std::array<double, 7> coriolis_array = model.coriolis(robot_state);
        std::array<double, 42> jacobian_array =
            model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
        log["jacobian"].push_back(jacobian_array);
        log["coriolis"].push_back(coriolis_array);
        log["gravity"].push_back(gravity_array);

        log["q"].push_back(robot_state.q);
        log["qd"].push_back(robot_state.dq);
        log["tau"].push_back(robot_state.tau_J);

        // convert to Eigen
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
        Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> tau(robot_state.tau_J.data());
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> tau_ext_filtered(robot_state.tau_ext_hat_filtered.data());
        Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
        Eigen::Vector3d position(transform.translation());
        Eigen::Quaterniond orientation(transform.linear());

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
        log["EE_T_K"].push_back(robot_state.EE_T_K);
        log["O_T_EE"].push_back(robot_state.O_T_EE);
        log["EE_position"].push_back({robot_state.O_T_EE[12], robot_state.O_T_EE[13],
                                      robot_state.O_T_EE[14]});
                                      
        // std::cout << "coriollis: " << coriolis.transpose() << std::endl;
        Eigen::Matrix<double, 6, 1> wrench = jacobian * (tau - gravity);
        std::cout << "wrench: " << wrench.transpose() << std::endl;
        log["wrench"].push_back({wrench[0], wrench[1], wrench[2], wrench[3], wrench[4], wrench[5]});

        Eigen::Vector3d linear_force = wrench.segment(0, 3);
        std::cout << "Linear force:  " << linear_force.transpose() << std::endl;
        log["force"].push_back({linear_force[0], linear_force[1], linear_force[2]});

        // std::cout << "tau_ext_hat_filtered: ";
        // print(robot_state.tau_ext_hat_filtered);
        // std::cout << "K_F_ext_hat_K: ";
        // print(robot_state.K_F_ext_hat_K);

        // std::cout << "O_F_ext_hat_K: ";
        // print(robot_state.O_F_ext_hat_K);

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
    std::cerr << "ERROR: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
