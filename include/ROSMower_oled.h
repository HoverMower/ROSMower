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
#include "sensor_msgs/NavSatFix.h"
#include "ublox_msgs/NavRELPOSNED9.h"

class ROSMower_oled
{
public:
    ROSMower_oled();
    ~ROSMower_oled();

    void batteryCallback(const rosmower_msgs::Battery::ConstPtr &msg);
    void perimeterCallback(const rosmower_msgs::PerimeterMsg::ConstPtr &msg);
    void gpsFixCallback(const sensor_msgs::NavSatFix::ConstPtr &msg);
    void gpsPosCallback(const ublox_msgs::NavRELPOSNED9::ConstPtr &msg);
    void update();

private:
    // Publishers
    ros::NodeHandle nh;
    ros::Publisher pub_oled;

    // Subscriber
    ros::Subscriber sub_battery;
    ros::Subscriber sub_perimeter;
    ros::Subscriber sub_gpsFix;
    ros::Subscriber sub_gpsPos;

    // Lines to display
    char line1[15] = "Startup ROS   ";
    char line2[15] = "ROSMOWER      ";
    char line3[15] = "V0.0.0        ";

    // other variables
    char peri_left_state = ' ';
    char peri_right_state = ' ';

    __int8_t gpsState = 0;
};

#endif
