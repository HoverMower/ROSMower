#include "ROSMower_ds4.hpp"
#include "ROSMower_SafetyController.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto ds4 = std::make_shared<ROSMower_ds4>("ROSMower_DS4");
    auto safetyController = std::make_shared<ROSMower_SafetyController>("ROSMower_safetyController");

    rclcpp::Rate rate(20.0);

    while (rclcpp::ok())
    {
        // DS4 controller gets startet by constructor

        // Safety Controller to monitor bumper and perimeter
        safetyController->run();
        rclcpp::spin(ds4);
        rclcpp::spin(safetyController);
        rate.sleep();
    }

    return 0;
}
