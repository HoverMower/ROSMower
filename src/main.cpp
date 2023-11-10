#include "ROSMower_joystick.hpp"
#include "ROSMower_SafetyController.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto joy = std::make_shared<ROSMower_joystick>("ROSMower_Joy");
    auto safetyController = std::make_shared<ROSMower_SafetyController>("ROSMower_safetyController");

    rclcpp::Rate rate(20.0);

    while (rclcpp::ok())
    {
        // Joy controller node gets startet by constructor

        // Safety Controller to monitor bumper and perimeter
        safetyController->run();
        rclcpp::spin(joy);
        rclcpp::spin(safetyController);
        rate.sleep();
    }

    return 0;
}
