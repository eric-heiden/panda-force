#include <Eigen/Dense>
#include <sstream>
#include <vector>
#include <zmq.hpp>

#include "state_recorder.hpp"
#include "settings.h"

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

static bool starts_with(const std::string &text, const std::string &query) {
  return text.rfind(query, 0) == 0;
}

int main(int argc, char **argv) {
  std::cout << "PANDA_HOME: " << PANDA_HOME << std::endl;

  // run ZeroMQ zerver which will take over the main thread
  zmq::context_t ctx;
  zmq::socket_t socket(ctx, zmq::socket_type::rep);
  socket.bind("tcp://*:5555");
  std::cout << "ZMQ server is waiting on port 5555..." << std::endl;

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

    
    franka::Robot *robot = nullptr;

    if (command == "connect") {
        std::string franka_address;
        ss >> franka_address;
        robot = new franka::Robot(franka_address);
        std::cout << "Connected to robot at " << franka_address << std::endl;
        response = "OK";
    }
    else if (command == "disconnect") {
        delete robot;
        robot = nullptr;
        response = "OK";
    }
    else if (command == "set_knife") {
      std::string knife_name;
        ss >> knife_name;
        if (knife_name == "edc") {
            settings::set_EE_properties(
        robot, std::string(PANDA_HOME) +
                   std::string("/edc-knife-endeffector-config.json"));
        }
    } else if (command == "estimation_step") {
      estimator->step();
      response = serialize(estimator->serialize_settings());
    } else if (command == "state_names") {
      response = serialize(sim.get_state_names());
    } else if (command == "param_names") {
      auto params = sim.get_parameters();
      std::vector<std::string> names(params.size());
      for (size_t i = 0; i < params.size(); ++i) {
        names[i] = params[i].name;
      }
      response = serialize(names);
    } else if (command == "param_guess") {
      response = serialize(estimator->problem.get_parameter_guess());
    } else if (command == "param_sample") {
      response = serialize(estimator->problem.get_parameter_sample());
    } else if (command == "state_dim") {
      response = std::to_string(sim.state_dim());
    } else if (command == "param_dim") {
      response = std::to_string(sim.param_dim());
    } else if (command == "particles") {
      response = serialize(estimator->particles);
    } else if (command == "rollout") {
      ss >> num_steps;
      int trajectory_id;
      ss >> trajectory_id;
      for (int i = 0; i < sim.param_dim(); ++i) {
        ss >> parameters[i];
      }
      trajectory.resize(num_steps);
      target_dist->project_parameters(parameters, parameters);
      sim.rollout(&trajectory, parameters, false, groundtruths, trajectory_id);
      response = serialize(trajectory);
    } else if (command == "logprob") {
      for (int i = 0; i < sim.param_dim(); ++i) {
        ss >> parameters[i];
      }
      double v = estimator->problem.logprob(parameters);
      response = serialize(v);
    } else if (command == "logprobgrad") {
      for (int i = 0; i < sim.param_dim(); ++i) {
        ss >> parameters[i];
      }
      const auto &v = estimator->problem.logprobgrad(parameters);
      response = serialize(v);
    } else if (command == "step") {
      // compute dynamics step given the previous state and parameters
      for (size_t i = 0; i < state_and_params.size(); ++i) {
        ss >> state_and_params[i];
      }
      step_functor(state_and_params, next_state);
      response = serialize(next_state);
    } else if (command == "step_jacobian") {
      // compute gradient of dynamics step given the previous state and
      // parameters
      static std::vector<double> step_jac(state_and_params.size() * 4);
      for (size_t i = 0; i < state_and_params.size(); ++i) {
        ss >> state_and_params[i];
      }
      step_functor.jacobian(state_and_params, step_jac);
      response = serialize(step_jac);
    } else if (command == "groundtruth") {
      int id;
      ss >> id;
      response = serialize(groundtruths[id]);
    } else if (command == "original_param_bounds") {
      auto params = sim.get_parameters();
      std::vector<double> bounds(params.size() * 2);
      for (size_t i = 0; i < params.size(); ++i) {
        bounds[2 * i] = params[i].minimum;
        bounds[2 * i + 1] = params[i].maximum;
      }
      response = serialize(bounds);
    } else if (command == "param_bounds") {
      const auto &limits = target_dist->get_param_limits();
      std::vector<double> bounds(limits.size() * 2);
      for (size_t i = 0; i < limits.size(); ++i) {
        bounds[2 * i] = limits[i].first;
        bounds[2 * i + 1] = limits[i].second;
      }
      response = serialize(bounds);
    } else if (command == "uses_parameter_normalization") {
      response = std::to_string(target_dist->uses_parameter_normalization());
    } else if (command == "project_params") {
      for (int i = 0; i < sim.param_dim(); ++i) {
        ss >> parameters[i];
      }
      std::vector<double> params_out;
      target_dist->project_parameters(parameters, params_out);
      response = serialize(params_out);
    } else if (command == "unproject_params") {
      for (int i = 0; i < sim.param_dim(); ++i) {
        ss >> parameters[i];
      }
      std::vector<double> params_out;
      target_dist->unproject_parameters(parameters, params_out);
      response = serialize(params_out);
    } else if (command == "ksd") {
      int num_particles;
      ss >> num_particles;
      std::vector<std::vector<double>> particles(num_particles);
      for (int j = 0; j < num_particles; ++j) {
        for (int i = 0; i < sim.param_dim(); ++i) {
          ss >> parameters[i];
        }
        particles[j] = parameters;
      }
      double bandwidth;
      ss >> bandwidth;
      const auto &[ksd, ksdgrads] =
          estimator->problem.ksd(particles, bandwidth);
      response = serialize(ksd) + " " + serialize(ksdgrads);
    }

    //  Send reply back to client
    zmq::message_t reply((void *)response.data(), response.size());
    socket.send(reply);
  }

  return EXIT_SUCCESS;
}