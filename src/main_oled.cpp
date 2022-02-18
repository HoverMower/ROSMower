#include "ROSMower_oled.h"
#include "ROSMower_ds4.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ROSMower_oled");

    ROSMower_oled oled;
    ROSMower_ds4 ds4;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Rate rate(10.0);
    int rate_counter = 0;

    while (ros::ok())
    {
        rate_counter++;
        if (rate_counter == 10)
        {
            oled.update();
            rate_counter = 0;
        }
        ds4.update();
        rate.sleep();
    }

    return 0;
}
