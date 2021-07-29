#include <ros/ros.h>
#include "perimeter.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "perimeter_driver");

    Perimeter perimeter;
    
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Rate rate(100.0);

    while (ros::ok()) {

        perimeter.read();
        rate.sleep();
    }

    return 0;
}
