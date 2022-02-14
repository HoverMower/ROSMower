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
int test = snprintf(line1, 15, "B %2.2f CA %2.2f\0", msg->battery_voltage, msg->charge_current);
//   line1 = "BatV: " + std::to_string(msg->battery_voltage);
}

void ROSMower_oled::update()
{
   oled_display_node::DisplayOutput output;
   output.actionType    = 2;
   output.row           = 2;
   output.column        = 1;
   output.numChars      = 15;
   output.attributes    = 0;
   output.text          = line1;

   pub_oled.publish(output);
}




