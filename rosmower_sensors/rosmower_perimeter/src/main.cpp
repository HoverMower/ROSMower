#include <ros/ros.h>
#include "perimeter.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "perimeter_driver");

    Perimeter perimeter;
    
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Time prev_time = ros::Time::now();
    ros::Rate rate(100.0);

    while (ros::ok()) {
        const ros::Time time = ros::Time::now();
        const ros::Duration period = time - prev_time;
        prev_time = time;

        perimeter.read();
        rate.sleep();
    }

    return 0;
}
