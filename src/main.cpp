#include "ROSMower_oled.h"
#include "ROSMower_ds4.h"
#include "ROSMower_SafetyController.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ROSMower");

    ROSMower_oled oled;
    ROSMower_ds4 ds4;
    ROSMower_SafetyController safetyController;

    ros::AsyncSpinner spinner(0);
    spinner.start();

    ros::Rate rate(10.0);
    int rate_counter = 0;

    while (ros::ok())
    {
        // DS4 controller gets startet by constructor

        // Safety Controller to monitor bumper and perimeter
        safetyController.run();

        // slow down oled update
        rate_counter++;
        if (rate_counter == 10)
        {
            oled.update();
            rate_counter = 0;
        }

        rate.sleep();
    }

    return 0;
}
