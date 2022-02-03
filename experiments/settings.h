#pragma once

#include "json.hpp"
#include <franka/robot.h>
#include <string>

namespace settings {
/**
 * Frequency at which the log is populated with robot state information.
 * If it is too high, libfranka will show the following reflex error:
 *
 * libfranka: Move command aborted: motion aborted by reflex!
 * ["communication_constraints_violation"]
 */
static const double kLogRate = 200.0;

static const std::string franka_address = "172.16.0.2";

void set_robot_parameters(franka::Robot &robot) {
  // Set additional parameters always before the control loop, NEVER in the
  // control loop! Set the joint impedance.
  robot.setJointImpedance({{8000, 8000, 8000, 7000, 7000, 6500, 6500}});

  // Set the collision behavior.
  std::array<double, 7> lower_torque_thresholds_nominal{
      {25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.}};
  std::array<double, 7> upper_torque_thresholds_nominal{
      {35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0}};
  std::array<double, 7> lower_torque_thresholds_acceleration{
      {25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.0}};
  std::array<double, 7> upper_torque_thresholds_acceleration{
      {35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0}};
  std::array<double, 6> lower_force_thresholds_nominal{
      {160.0, 160.0, 160.0, 150.0, 150.0, 150.0}};
  std::array<double, 6> upper_force_thresholds_nominal{
      {180.0, 180.0, 180.0, 170.0, 170.0, 170.0}};
  std::array<double, 6> lower_force_thresholds_acceleration{
      {160.0, 160.0, 160.0, 150.0, 150.0, 150.0}};
  std::array<double, 6> upper_force_thresholds_acceleration{
      {180.0, 180.0, 180.0, 170.0, 170.0, 170.0}};
  robot.setCollisionBehavior(
      lower_torque_thresholds_acceleration,
      upper_torque_thresholds_acceleration, lower_torque_thresholds_nominal,
      upper_torque_thresholds_nominal, lower_force_thresholds_acceleration,
      upper_force_thresholds_acceleration, lower_force_thresholds_nominal,
      upper_force_thresholds_nominal);
}

/**
 * Sets a default collision behavior, joint impedance, Cartesian impedance, and
 * filter frequency. Taken from libfranka examples.
 *
 * @param[in] robot Robot instance to set behavior on.
 */
void set_default_behavior(franka::Robot &robot) {
  robot.setCollisionBehavior({{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
                             {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
                             {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
                             {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
                             {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
                             {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
                             {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
                             {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});
  robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
  // these are the highest possible Cartesian impedance settings!
  robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
}

void set_EE_properties(franka::Robot &robot,
                       const std::string &config_filename) {
  std::ifstream config_file(config_filename);
  nlohmann::json config = nlohmann::json::parse(config_file);
  std::cout << "Setting EE properties from " << config_filename << std::endl;
  double mass = config["mass"];
  std::array<double, 3> com = config["centerOfMass"];
  std::array<double, 9> inertia = config["inertia"];
  std::array<double, 16> transform = config["transformation"];

  std::cout << "Mass:\t" << mass << std::endl;

  std::cout << "COM:\t" << com[0] << "\t" << com[1] << "\t" << com[2]
            << std::endl;
  std::cout << "Inertia: " << std::endl;
  for (int i = 0; i < 3; ++i) {
    std::cout << "\t";
    for (int j = 0; j < 3; ++j) {
      // column-major format
      std::cout << inertia[j * 3 + i] << "\t";
    }
    std::cout << std::endl;
  }

  std::cout << "Transform:\n";
  for (int i = 0; i < 4; ++i) {
    std::cout << "\t";
    for (int j = 0; j < 4; ++j) {
      // column-major format
      std::cout << transform[j * 4 + i] << "\t";
    }
    std::cout << std::endl;
  }

  robot.setLoad(mass, com, inertia);
  robot.setEE(transform);
}
} // namespace settings