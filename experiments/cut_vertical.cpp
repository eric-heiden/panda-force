#include "vertical_cutting.hpp"

int main(int argc, char **argv)
{
    //   if (argc != 2) {
    //     std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    //     return -1;
    // }

    franka::Robot robot(settings::franka_address);
    franka::Model model = robot.loadModel();

    std::string log_filename = "log_panda_cut_vertical_large_potato.json";

    VerticalCutting cutting(robot, model, log_filename);

    double time_max = 1.7;

    // vertical cutting velocity
    const double v_z = -0.05;

    // First move the robot to a suitable joint configuration
    // std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    std::array<double, 7> q_goal = {{0.000834242574274213,
                                     0.2900282463107192,
                                     0.0005799547732569646,
                                     -2.937202012078804,
                                     0.0035845843508754173,
                                     3.226281019502349,
                                     0.7825557258720371}};

    cutting.initiate(q_goal);

    std::cout << "WARNING: This example will move the robot!\n\nTHIS PROGRAM IS FOR THE LARGE (SLICING) KNIFE.\n\n"
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    return cutting.run(time_max, v_z);
}
