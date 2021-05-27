#pragma once

#include <cmath>
#include <iostream>
#include <atomic>
#include <mutex>
#include <thread>

#include <franka/exception.h>
#include <franka/robot.h>

#include "examples_common.h"

#include "state_recorder.hpp"
#include "settings.h"
#include "utils.hpp"

struct VerticalCutting
{
    std::atomic_bool running{false};

    franka::Robot &robot;
    franka::Model &model;
    StateRecorder state_recorder;

    std::array<double, 7> q_goal{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};

private:
    bool initiated{false};
    struct
    {
        std::mutex mutex;
        bool has_data;
        bool success{true};
        franka::RobotState robot_state;
    } print_data{};

public:
    VerticalCutting(franka::Robot &robot, franka::Model &model, const std::string &log_filename) : robot(robot), model(model), state_recorder(log_filename, model)
    {
    }

    void initiate(const std::array<double, 7> &q_goal)
    {
        this->q_goal = q_goal;

        settings::set_default_behavior(robot);
        settings::set_robot_parameters(robot);

        initiated = true;
        std::cout << "Cutting initiated.\n";
    }

    int run(double time_max, double v_z)
    {
        if (!initiated)
        {
            std::cerr << "Not initiated.\n";
            return EXIT_FAILURE;
        }
        running = true;

        std::thread print_thread([this]() {
            while (running)
            {
                // Sleep to achieve the desired print rate.
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(static_cast<int>(1000.0 / settings::kLogRate)));

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
        });

        try
        {
            MotionGenerator motion_generator(0.2, q_goal);
            robot.control(motion_generator);
            std::cout << "Finished moving to initial joint configuration." << std::endl;

            std::cout
                << "Press Enter to continue..." << std::endl;
            std::cin.ignore();

            double time = 0.0;
            robot.control([=, &time](const franka::RobotState &state,
                                     franka::Duration period) -> franka::CartesianVelocities {
                time += period.toSec();

                if (print_data.mutex.try_lock())
                {
                    print_data.has_data = true;
                    print_data.robot_state = state;
                    print_data.mutex.unlock();
                    // if (std::abs(time - time_max / 2.0) < 0.05)
                    // {
                    //     std::cout << "Joint q:";
                    //     for (size_t i = 0; i < state.q.size(); ++i)
                    //     {
                    //         std::cout << "  " << state.q[i];
                    //     }
                    //     std::cout << "\n";
                    // }
                }

                double v = v_z * (sinus_step(time, 1.0)); // + sinus_step(time_max - time, 0.2) - 1.0);
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
            if (print_data.mutex.try_lock())
            {
                print_data.has_data = true;
                print_data.success = false;
                print_data.mutex.unlock();
            }
            std::cerr << e.what() << std::endl;
            if (print_thread.joinable())
            {
                print_thread.join();
            }

            state_recorder.save();
            return EXIT_FAILURE;
        }
        state_recorder.save();

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

        state_recorder.save();

        return EXIT_SUCCESS;
    }
};