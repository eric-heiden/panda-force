
#include <ctime>
#include <fstream>

#include <Eigen/Dense>

#include <franka/model.h>
#include <franka/robot.h>

#include "json.hpp"

struct StateRecorder {
  const std::string filename;
  const franka::Model &model;

  typedef std::chrono::high_resolution_clock Time;
  typedef std::chrono::milliseconds ms;

  /**
   * Whether to keep recording at maximum frequency.
   */
  bool continuous_recording{false};

  bool print_to_stdout{false};

  int save_every_nth_update{0};

  nlohmann::json log;

  std::vector<franka::RobotState> states;
  std::vector<double> times;
  int step{0};
  mutable int another_step{0};

  StateRecorder(const std::string &filename, const franka::Model &model,
                size_t stack_size = 100000)
      : filename(filename), model(model) {
    std::time_t time = std::time(nullptr);
    log["datetime"] = std::ctime(&time);

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

    states.reserve(stack_size);
    times.reserve(stack_size);
  }

  bool operator()(const franka::RobotState &robot_state) {
    states.push_back(robot_state);
    // static auto t0 = Time::now();
    static int counter = 0;
    static double t0 = robot_state.time.toSec();

    // std::chrono Time::now() seems to cause a lot of delay (system call)
    // auto ti = Time::now();
    // std::chrono::duration<double> fs = ti - t0;
    // auto d = std::chrono::duration_cast<std::chrono::microseconds>(fs);
    // d.count() / 1e6;
    double time = robot_state.time.toSec() - t0;
    times.push_back(time);
    // if (counter % 10 == 0) {
    //   // std::cout << "time: " << time << "\n";
    //   std::cout << "time: " << time << "  step: " << another_step << "
    //   counter: " << counter << "  StateRecorder: " << this << "\n";
    // }

    // if (print_to_stdout)
    // {
    //     std::cout << "time: " << time << std::endl;
    //     std::cout << "wrench: " << wrench.transpose() << std::endl;
    //     std::cout << "Linear force:  " << linear_force.transpose() <<
    //     std::endl;
    //     // std::cout << "tau_ext_hat_filtered: ";
    //     // print(robot_state.tau_ext_hat_filtered);
    //     // std::cout << "K_F_ext_hat_K: ";
    //     // print(robot_state.K_F_ext_hat_K);

    //     // std::cout << "O_F_ext_hat_K: ";
    //     // print(robot_state.O_F_ext_hat_K);

    //     std::cout << std::endl;
    // }
    ++counter;
    this->another_step = counter;

    return continuous_recording;
  }

  void process() {
    const int progress_bar_width = 64;

    for (size_t i = 0; i < states.size(); ++i) {
      if (states.size() > 5000) {
        std::cout << "Saving " << states.size() << " states [";
        double progress = double(i) / (states.size() - 1.0);
        int pos = progress_bar_width * progress;
        for (int i = 0; i < progress_bar_width; ++i) {
          if (i < pos)
            std::cout << "=";
          else if (i == pos)
            std::cout << ">";
          else
            std::cout << " ";
        }
        std::cout << "] " << int(progress * 100.0) << " %\r";
        std::cout.flush();
      }

      const auto &robot_state = states[i];
      double time = times[i];

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
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> gravity(
          gravity_array.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(
          coriolis_array.data());
      Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(
          jacobian_array.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> tau(
          robot_state.tau_J.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> tau_ext_filtered(
          robot_state.tau_ext_hat_filtered.data());
      Eigen::Affine3d transform(
          Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
      Eigen::Vector3d position(transform.translation());
      Eigen::Quaterniond orientation(transform.linear());

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
      log["EE_position"].push_back({robot_state.O_T_EE[12],
                                    robot_state.O_T_EE[13],
                                    robot_state.O_T_EE[14]});

      // std::cout << "coriollis: " << coriolis.transpose() << std::endl;
      Eigen::Matrix<double, 6, 1> wrench = jacobian * (tau - gravity);
      log["EE_wrench"].push_back(
          {wrench[0], wrench[1], wrench[2], wrench[3], wrench[4], wrench[5]});

      Eigen::Vector3d linear_force = wrench.segment(0, 3);
      log["EE_linear_force"].push_back(
          {linear_force[0], linear_force[1], linear_force[2]});

      Eigen::Matrix<double, 6, 1> twist = jacobian * dq;
      log["EE_twist"].push_back(
          {twist[0], twist[1], twist[2], twist[3], twist[4], twist[5]});
    }
    std::cout << std::endl;
  }

  void save() {
    // states.resize(step);
    // times.resize(step);
    process();
    std::ofstream file(filename);
    file << log;
    std::cout << "Saved " << states.size() << " states to log file at "
              << filename << "\n";
  }
};