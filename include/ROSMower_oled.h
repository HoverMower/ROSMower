#ifndef _ROSMOWER_OLED_H
#define _ROSMOWER_OLED_H

#include <ros/ros.h>
#include <string.h>
#include <stdio.h>
#include "rosmower_msgs/PerimeterMsg.h"
#include "rosmower_msgs/Bumper.h"
#include "rosmower_msgs/Battery.h"
#include "rosmower_msgs/MowMotor.h"
#include "oled_display_node/DisplayOutput.h"
#include "std_msgs/Int32.h"

class ROSMower_oled
{
public:
    ROSMower_oled();
    ~ROSMower_oled();

    void batteryCallback(const rosmower_msgs::Battery::ConstPtr& msg);
    void update();

private:
                           
    // Publishers
    ros::NodeHandle nh;
    ros::Publisher pub_oled;

    // Subscriber
    ros::Subscriber sub_battery;

    // Lines to display
    char line1[15] = "Hello Text    ";

};

#endif
