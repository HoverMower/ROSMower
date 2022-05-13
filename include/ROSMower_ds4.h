#ifndef _ROSMOWER_DS4_H
#define _ROSMOWER_DS4_H

#include <ros/ros.h>
#include <string.h>
#include <stdio.h>
#include "std_msgs/Bool.h"
#include "ds4_driver/Status.h"
#include "rosmower_msgs/pressSwitch.h"
#include "rosmower_msgs/setSwitch.h"
#include "rosmower_msgs/setMowMotor.h"
#include "rosmower_msgs/Switches.h"
#include "rosmower_msgs/MowMotor.h"
#include "geometry_msgs/PointStamped.h"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

class ROSMower_ds4
{
public:
    ROSMower_ds4();
    ~ROSMower_ds4();

    void ds4Callback(const ds4_driver::Status::ConstPtr &msg);
    void mowCallback(const rosmower_msgs::MowMotor::ConstPtr &msg);
    void switchesCallback(const rosmower_msgs::Switches::ConstPtr &msg);
    void update();

private:

    
    void publish_point();
    // Publishers
    ros::NodeHandle nh;
    ros::Publisher pub_eStop;
    ros::Publisher pub_point;

    // Service Clients
    ros::ServiceClient _srv_pressSwitch = nh.serviceClient<rosmower_msgs::pressSwitch>("hovermower/pressSwitch");
    ros::ServiceClient _srv_setSwitch = nh.serviceClient<rosmower_msgs::setSwitch>("hovermower/setSwitch");
    ros::ServiceClient _srv_mow = nh.serviceClient<rosmower_msgs::setMowMotor>("hovermower/setMowMotorSpeed");

    // Subscriber
    ros::Subscriber _sub_ds4;
    ros::Subscriber _sub_mow;
    ros::Subscriber _sub_switches;

    // tf
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener* tfListener;
    

    uint8_t _switch1;
    uint8_t _switch2;
    int _mow_speed;

    // keep last button state to not toggle too often (debounce)
    int _ds4_last_button_l1 = 0;
    int _ds4_last_button_l2 = 0;
    int _ds4_last_button_r1 = 0;
    int _ds4_last_button_circle = 0;
    int _ds4_last_button_square = 0;
    int _ds4_last_button_triangle = 0;

    bool _e_stop;

};

#endif
