#include <cmath>
#include <iostream>
#include <atomic>
#include <mutex>
#include <thread>

#include <franka/exception.h>
#include <franka/robot.h>

#include "examples_common.h"
#include "state_recorder.hpp"

#include "utils.hpp"

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

    StateRecorder state_recorder("log_panda_cut_vertical_potato.json", model);

    const double print_rate = 500.0;
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
    // First move the robot to a suitable joint configuration
    // std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    std::array<double, 7> q_goal = {{0.000834242574274213,
                                     0.2900282463107192,
                                     0.0005799547732569646,
                                     -2.937202012078804,
                                     0.0035845843508754173,
                                     3.226281019502349,
                                     0.7825557258720371}};

    try
    {

        MotionGenerator motion_generator(0.2, q_goal);
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

        double time_max = 6.1;
        double time = 0.0;
        const double v_z = -0.05;
        const double kappa = 2.0;
        robot.control([=, &time, &print_data, &running](const franka::RobotState &state,
                                                        franka::Duration period) -> franka::CartesianVelocities {
            time += period.toSec();

            if (print_data.mutex.try_lock())
            {
                print_data.has_data = true;
                print_data.robot_state = state;
                print_data.mutex.unlock();
            }

            double v = v_z * (sinus_step(time, 0.2) + sinus_step(time_max - time, 0.2) - 1.0);
            // double v = -0.001;
            // std::cout << v << "\n";
            franka::CartesianVelocities output = {{0.0, 0.0, v, 0.0, 0.0, 0.0}};
            if (time >= time_max)
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

    try
    {
        MotionGenerator motion_generator(0.2, q_goal);
        std::cout << "Moving back to start configuration..." << std::endl;
        robot.control(motion_generator);
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
