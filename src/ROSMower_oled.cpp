#include "ROSMower_oled.h"

ROSMower_oled::ROSMower_oled()
{

    // Register  publisher
    pub_oled = nh.advertise<oled_display_node::DisplayOutput>("display_node", 3);
    sub_battery = nh.subscribe("sensors/Battery", 1000, &ROSMower_oled::batteryCallback, this);
}

ROSMower_oled::~ROSMower_oled()
{
    
}

ROSMower_oled::batteryCallback(const rosmower_msgs::BatteryMsg::ConstPtr& msg)
{
   oled_display_node::DisplayOutput msg;
   msg.row = 0;
   msg.column = 0;
   msg.text ="BatVCC: %d

   ROS_INFO("I heard: [%s]", msg->data.c_str());
}




