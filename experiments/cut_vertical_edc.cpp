#include "vertical_cutting.hpp"

int main(int argc, char **argv)
{
    //   if (argc != 2) {
    //     std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    //     return -1;
    // }

    franka::Robot robot(settings::franka_address);
    franka::Model model = robot.loadModel();

    std::string log_filename = "log_panda_cut_vertical_edc_potato.json";

    VerticalCutting cutting(robot, model, log_filename);

    double time_max = 1.85;

    // vertical cutting velocity
    const double v_z = -0.05;

    std::array<double, 7> q_goal = {{0.0004674389429623014,
                                     0.37481192141917014,
                                     0.0012269206470816648,
                                     -2.911198324045812,
                                     0.0023427635109394586,
                                     3.2852705442232546,
                                     0.7835071256704961}};

    cutting.initiate(q_goal);

    std::cout << "WARNING: This example will move the robot!\n\nTHIS PROGRAM IS FOR THE SMALL (EDC) KNIFE.\n\n"
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    return cutting.run(time_max, v_z);
}
