#include "ROSMower_oled.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ROSMower_oled");

    ROSMower_oled oled;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Rate rate(1.0);
    int rate_counter = 0;

    while (ros::ok())
    {
oled.update();
        rate.sleep();
    }

    return 0;
}
