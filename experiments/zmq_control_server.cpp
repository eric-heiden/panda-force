#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <vector>

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>

#include <Eigen/Dense>

#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

#include <zmq.hpp>

#include "spline.hpp"

#include "examples_common.h"
#include "settings.h"
#include "state_recorder.hpp"

using namespace std::chrono_literals;
namespace fs = std::filesystem;

const int progress_bar_width = 64;

static inline std::string serialize(double d) {
  std::stringstream ss;
  ss << std::setprecision(std::numeric_limits<double>::digits10) << d;
  return ss.str();
}

static inline std::string serialize(const std::string &s) {
  return "\"" + s + "\"";
}

static inline std::string serialize(const std::vector<double> &v) {
  std::stringstream ss;
  ss << "[";
  for (size_t i = 0; i < v.size(); ++i) {
    ss << std::setprecision(std::numeric_limits<double>::digits10) << v[i];
    if (i < v.size() - 1) {
      ss << " ";
    }
  }
  ss << "]";
  return ss.str();
}

template <typename T, size_t N>
static inline std::string serialize(const std::array<T, N> &v) {
  std::stringstream ss;
  ss << "[";
  for (size_t i = 0; i < v.size(); ++i) {
    ss << serialize(v[i]);
    if (i < v.size() - 1) {
      ss << " ";
    }
  }
  ss << "]";
  return ss.str();
}

static inline std::string serialize(const Eigen::VectorXd &v) {
  std::stringstream ss;
  ss << "[";
  for (size_t i = 0; i < v.size(); ++i) {
    ss << std::setprecision(std::numeric_limits<double>::digits10) << v[i];
    if (i < v.size() - 1) {
      ss << " ";
    }
  }
  ss << "]";
  return ss.str();
}

template <typename T>
static inline std::string serialize(const std::vector<T> &v) {
  std::stringstream ss;
  ss << "[";
  for (size_t i = 0; i < v.size(); ++i) {
    ss << serialize(v[i]);
    if (i < v.size() - 1) {
      ss << " ";
    }
  }
  ss << "]";
  return ss.str();
}

static inline std::string
serialize(const std::map<std::string, std::string> &m) {
  std::stringstream ss;
  size_t i = 0;
  for (const auto &[key, value] : m) {
    ss << key << ": " << value;
    if (i++ < m.size() - 1) {
      ss << ", ";
    }
  }
  return ss.str();
}

namespace std {
template <size_t N>
static std::istream &operator>>(std::istream &in, std::array<double, N> &arr) {
  for (size_t i = 0; i < N; ++i) {
    in >> arr[i];
  }
  return in;
}
} // namespace std

static bool starts_with(const std::string &text, const std::string &query) {
  return text.rfind(query, 0) == 0;
}

std::string get_time_str() {
  auto t = std::time(nullptr);
  auto tm = *std::localtime(&t);
  std::ostringstream oss;
  oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
  return oss.str();
}

std::string uppercase(const std::string &s) {
  std::string r = s;
  for (size_t i = 0; i < s.size(); ++i) {
    r[i] = toupper(s[i]);
  }
  return r;
}

std::condition_variable cv;
// char entered_value;
void read_value() {
  std::cin.ignore();
  cv.notify_one();
}
void prompt_confirmation(const std::string &knife) {
  std::cout << "WARNING: The robot is about to move!\n\n";
  if (knife.empty()) {
    std::cout << "NO KNIFE HAS BEEN SELECTED.\n\n";
  } else {
    std::cout << "THE SELECTED KNIFE IS THE " << uppercase(knife)
              << " KNIFE.\n\n";
  }
  std::cout << "Please make sure to have the user stop button at hand!"
            << std::endl
            << "You have 5 seconds to press Enter to continue, "
            << "the server will terminate otherwise..." << std::endl;
  // std::cin.ignore();
  std::thread th(read_value);
  std::mutex mtx;
  std::unique_lock<std::mutex> lck(mtx);
  while (cv.wait_for(lck, std::chrono::seconds(5)) == std::cv_status::timeout) {
    std::cout << "Time out. Shutting down control server.\n";
    std::exit(0);
  }
  th.join();
}

int main(int argc, char **argv) {
  std::cout << "PANDA_HOME: " << PANDA_HOME << std::endl;

  // run ZeroMQ zerver which will take over the main thread
  zmq::context_t ctx;
  zmq::socket_t socket(ctx, zmq::socket_type::rep);
  socket.bind("tcp://*:5555");
  std::cout << "ZMQ server is waiting on port 5555..." << std::endl;

  std::unique_ptr<franka::Robot> robot;
  std::unique_ptr<franka::Model> model;

  std::string knife_selection = "";

  std::string response;
  while (true) {
    zmq::message_t request;
    response = "";

    //  Wait for next request from client
    socket.recv(&request);
    std::string s = request.to_string();
    std::cout << "Received request: " << s << std::endl;

    std::stringstream ss(s);
    std::string command;
    ss >> command;

    if (command == "connect") {
      std::string franka_address;
      ss >> franka_address;
      robot = std::make_unique<franka::Robot>(franka_address);
      model = std::make_unique<franka::Model>(robot->loadModel());
      std::cout << "Connected to robot at " << franka_address << std::endl;
      response = "OK";
    } else if (command == "disconnect") {
      robot.reset();
      response = "OK";
    } else if (command == "error_recovery") {
      try {
        robot->automaticErrorRecovery();
        response = "OK";
      } catch (const std::exception &e) {
        response = e.what();
      }
    } else if (command == "set_knife") {
      if (!robot) {
        // throw std::runtime_error("Robot has not been connected!");
        response = "Robot has not been connected!";
      } else {
        std::string knife_name;
        ss >> knife_name;
        if (knife_name == "edc") {
          settings::set_EE_properties(
              *robot, std::string(PANDA_HOME) +
                          std::string("/edc-knife-endeffector-config.json"));
          knife_selection = knife_name;
          response = "OK";
        } else if (knife_name == "slicing") {
          settings::set_EE_properties(
              *robot,
              std::string(PANDA_HOME) +
                  std::string("/slicing-knife-endeffector-config.json"));
          knife_selection = knife_name;
          response = "OK";
        } else {
          // throw std::runtime_error("Unknown knife \"" + knife_name + "\"");
          response = "Unknown knife \"" + knife_name + "\"";
        }
      }
    } else if (command == "get_knife") {
      if (knife_selection.size() == 0) {
        response = "No knife has been selected.";
      } else {
        response = "OK " + knife_selection;
      }
    } else if (command == "record") {
      if (!robot) {
        // throw std::runtime_error("Robot has not been connected!");
        response = "Robot has not been connected!";
      } else {
        size_t num_steps;
        ss >> num_steps;
        int dt_in_ms;
        ss >> dt_in_ms;

        std::chrono::duration<int, std::milli> duration(dt_in_ms);

        std::string filename = std::string(PANDA_HOME) + "/log/" +
                               get_time_str() + "_recording.json";
        filename = fs::absolute(fs::path(filename));

        StateRecorder state_recorder(filename, *model);

        for (size_t t = 0; t < num_steps; ++t) {
          double progress = double(t) / (num_steps - 1.0);
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

          robot->read([&state_recorder](const franka::RobotState &robot_state) {
            state_recorder(robot_state);
            return false;
          });
          std::this_thread::sleep_for(duration);
        }
        state_recorder.save();
        response = "OK " + filename;
      }
    } else if (command == "retrieve") {
      std::string log_filename;
      ss >> log_filename;
      std::ifstream log(log_filename);
      if (log.bad() || log.fail()) {
        response = "Could not open file \"" + log_filename + "\"";
      } else {
        std::stringstream buffer;
        buffer << "OK ";
        buffer << log.rdbuf();
        response = buffer.str();
      }
    } else if (command == "move_to_q") {
      if (!robot) {
        // throw std::runtime_error("Robot has not been connected!");
        response = "Robot has not been connected!";
        goto send_response;
      }
      std::array<double, 7> q;
      ss >> q;
      double duration;
      ss >> duration;
      int confirm;
      ss >> confirm;
      int record;
      ss >> record;
      int record_frequency;
      ss >> record_frequency;
      if (record_frequency <= 0) {
        response = "Record frequency must be greater than zero!";
        goto send_response;
      }
      std::cout << "Go to joint position ";
      for (int i = 0; i < 7; ++i) {
        std::cout << q[i] << "  ";
      }
      std::cout << "over a duration of " << duration << " sec." << std::endl;

      if (confirm) {
        prompt_confirmation(knife_selection);
      }

      MotionGenerator motion_generator(duration, q);
      try {
        if (record) {
          std::string filename = std::string(PANDA_HOME) + "/log/" +
                                 get_time_str() + "_move_to_q.json";
          filename = fs::absolute(fs::path(filename));
          StateRecorder state_recorder(filename, *model);
          int step = 0;
          robot->control(
              [&motion_generator, &state_recorder, &record_frequency,
               &step](const franka::RobotState &robot_state,
                      franka::Duration period) -> franka::JointPositions {
                if (step % record_frequency == 0) {
                  state_recorder(robot_state);
                }
                ++step;
                return motion_generator(robot_state, period);
              });
          state_recorder.save();
          response = "OK " + filename;
        } else {
          robot->control(motion_generator);
          response = "OK";
        }
        std::cout << "Finished moving to joint configuration." << std::endl;
      } catch (const franka::ControlException &ex) {
        response = ex.what();
        std::cerr << "Error: " << ex.what() << std::endl;
      }
    } else if (command == "interpolate") {
      int num_waypoints;
      ss >> num_waypoints;
      if (num_waypoints < 3) {
        response = "At least 3 waypoints must be specified!";
        goto send_response;
      }
      std::array<std::vector<double>, 7> qs;
      for (int i = 0; i < 7; ++i) {
        qs[i].resize(num_waypoints);
      }
      for (int i = 0; i < 7 * num_waypoints; ++i) {
        ss >> qs[i % 7][i / 7];
      }
      double source_dt;
      ss >> source_dt;
      if (source_dt <= 0.0) {
        response = "Source timestep must be greater than zero!";
        goto send_response;
      }
      double target_dt;
      ss >> target_dt;
      if (target_dt <= 0.0) {
        response = "Target timestep must be greater than zero!";
        goto send_response;
      }
      std::vector<double> times(num_waypoints);
      for (int i = 0; i < num_waypoints; ++i) {
        times[i] = source_dt * i;
      }
      std::vector<tk::spline> splines;
      tk::spline::spline_type type = tk::spline::cspline;
      bool make_monotonic = false;
      for (int i = 0; i < 7; ++i) {
        // set first derivative boundaries to zero
        splines.push_back(tk::spline(times, qs[i], type, make_monotonic,
                                     tk::spline::first_deriv, 0.0,
                                     tk::spline::first_deriv, 0.0));
      }
      int num_target_waypoints = times[times.size() - 1] / target_dt;
      std::array<std::vector<double>, 7> target_qs;
      for (int i = 0; i < 7; ++i) {
        target_qs[i].resize(num_target_waypoints);
      }
      for (int t = 0; t < num_target_waypoints; ++t) {
        for (int i = 0; i < 7; ++i) {
          target_qs[i][t] = splines[i](t * target_dt);
        }
      }
      response = "OK " + serialize(target_qs);
    } else if (command == "follow_qs") {
      int num_waypoints;
      ss >> num_waypoints;
      if (num_waypoints < 3) {
        response = "At least 3 waypoints must be specified!";
        goto send_response;
      }
      std::array<std::vector<double>, 7> qs;
      for (int i = 0; i < 7; ++i) {
        qs[i].resize(num_waypoints);
      }
      for (int i = 0; i < 7 * num_waypoints; ++i) {
        ss >> qs[i % 7][i / 7];
      }
      double source_dt;
      ss >> source_dt;
      if (source_dt <= 0.0) {
        response = "Source timestep must be greater than zero!";
        goto send_response;
      }
      int record_frequency;
      ss >> record_frequency;
      if (record_frequency <= 0) {
        response = "Record frequency must be greater than zero!";
        goto send_response;
      }

      std::vector<double> times(num_waypoints);
      for (int i = 0; i < num_waypoints; ++i) {
        times[i] = source_dt * i;
      }
      std::vector<tk::spline> splines;
      tk::spline::spline_type type = tk::spline::cspline;
      bool make_monotonic = false;
      for (int i = 0; i < 7; ++i) {
        // set first derivative boundaries to zero
        splines.push_back(tk::spline(times, qs[i], type, make_monotonic,
                                     tk::spline::first_deriv, 0.0,
                                     tk::spline::first_deriv, 0.0));
      }
      prompt_confirmation(knife_selection);
      std::string filename = std::string(PANDA_HOME) + "/log/" +
                             get_time_str() + "_follow_qs.json";
      filename = fs::absolute(fs::path(filename));
      StateRecorder state_recorder(filename, *model);
      int step = 0;
      std::array<double, 7> current_q;
      double current_time = 0.0;
      const double end_time = times.back();
      try {
        robot->control([&splines, &state_recorder, &record_frequency, &step,
                        &current_q, &current_time, &end_time](
                           const franka::RobotState &robot_state,
                           franka::Duration period) -> franka::JointPositions {
          if (step % record_frequency == 0) {
            state_recorder(robot_state);
          }
          ++step;
          for (int i = 0; i < 7; ++i) {
            current_q[i] = splines[i](current_time);
          }
          current_time += period.toSec();
          franka::JointPositions output(current_q);
          output.motion_finished = current_time > end_time;
          return output;
        });
        state_recorder.save();
        response = "OK " + filename;
        std::cout << "Completed joint position motion.\n";
      } catch (const std::exception &e) {
        state_recorder.save();
        response = "FAIL " + filename + " Error at time " +
                   std::to_string(current_time) + ": " + e.what();
      }
    } else if (command == "follow_cartesian_vel") {
      int num_waypoints;
      ss >> num_waypoints;
      if (num_waypoints < 2) {
        response = "At least 2 waypoints must be specified!";
        goto send_response;
      }
      std::array<std::vector<double>, 6> qs;
      for (int i = 0; i < 6; ++i) {
        qs[i].resize(num_waypoints);
      }
      for (int i = 0; i < 6 * num_waypoints; ++i) {
        ss >> qs[i % 6][i / 6];
      }
      double source_dt;
      ss >> source_dt;
      if (source_dt <= 0.0) {
        response = "Source timestep must be greater than zero!";
        goto send_response;
      }
      int record_frequency;
      ss >> record_frequency;
      if (record_frequency <= 0) {
        response = "Record frequency must be greater than zero!";
        goto send_response;
      }

      std::vector<double> times(num_waypoints);
      for (int i = 0; i < num_waypoints; ++i) {
        times[i] = source_dt * i;
      }
      std::vector<tk::spline> splines;
      tk::spline::spline_type type = tk::spline::cspline;
      bool make_monotonic = false;
      for (int i = 0; i < 6; ++i) {
        // set first derivative boundaries to zero
        splines.push_back(tk::spline(times, qs[i], type, make_monotonic,
                                     tk::spline::first_deriv, 0.0,
                                     tk::spline::first_deriv, 0.0));
      }
      prompt_confirmation(knife_selection);
      std::string filename = std::string(PANDA_HOME) + "/log/" +
                             get_time_str() + "_follow_cartesian_vel.json";
      filename = fs::absolute(fs::path(filename));
      StateRecorder state_recorder(filename, *model);
      int step = 0;
      std::array<double, 6> current_vel;
      double current_time = 0.0;
      const double end_time = times.back();
      try {
        robot->control(
            [&splines, &state_recorder, &record_frequency, &step, &current_vel,
             &current_time, &end_time](
                const franka::RobotState &robot_state,
                franka::Duration period) -> franka::CartesianVelocities {
              if (step % record_frequency == 0) {
                state_recorder(robot_state);
              }
              ++step;
              for (int i = 0; i < 6; ++i) {
                current_vel[i] = splines[i](current_time);
              }
              current_time += period.toSec();
              franka::CartesianVelocities output(current_vel);
              output.motion_finished = current_time > end_time;
              return output;
            });
        state_recorder.save();
        response = "OK " + filename;
        std::cout << "Completed Cartesian velocity motion.\n";
      } catch (const std::exception &e) {
        state_recorder.save();
        response = "FAIL " + filename + " Error at time " +
                   std::to_string(current_time) + ": " + e.what();
      }
    } else {
      response = "!Unknown command \"" + command + "\"";
    }

  send_response:
    //  Send reply back to client
    zmq::message_t reply((void *)response.data(), response.size());
    socket.send(reply);
  }

  return EXIT_SUCCESS;
}