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

void ROSMower_oled::batteryCallback(const rosmower_msgs::Battery::ConstPtr& msg)
{
   line1 = "BatVCC: " + std::to_string(msg->battery_voltage);
}

void ROSMower_oled::update()
{
   oled_display_node::DisplayOutput output;
   output.actionType    = 2;
   output.row           = 3;
   output.column        = 1;
   output.numChars      = 15;
   output.attributes    = 0;
   output.text          = line1;

   pub_oled.publish(output);
}




