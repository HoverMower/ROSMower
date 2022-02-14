#include "ROSMower_oled.h"

ROSMower_oled::ROSMower_oled()
{

    // Register  publisher
    pub_oled = nh.advertise<oled_display_node::DisplayOutput>("display_node", 3);
    sub_battery = nh.subscribe("sensors/Battery", 1000, &ROSMower_oled::batteryCallback, this);
    sub_perimeter = nh.subscribe("sensors/Perimeter", 1000, &ROSMower_oled::perimeterCallback, this);
}

ROSMower_oled::~ROSMower_oled()
{
}

void ROSMower_oled::batteryCallback(const rosmower_msgs::Battery::ConstPtr &msg)
{
    switch (msg->charge_status)
    {
    case rosmower_msgs::Battery::CHARGE_STATUS_NOT_CONNECTED:
        snprintf(line1, 15, "%2.2fV            ", msg->battery_voltage);
        break;

    case rosmower_msgs::Battery::CHARGE_STATUS_STATION:
        snprintf(line1, 15, "%2.2fV STATION    ", msg->battery_voltage);
        break;

    case rosmower_msgs::Battery::CHARGE_STATUS_CHARGING:
        snprintf(line1, 15, "%2.2fV %2.2fA     ", msg->battery_voltage, msg->charge_current);
        break;
    }
}

void ROSMower_oled::perimeterCallback(const rosmower_msgs::PerimeterMsg::ConstPtr &msg)
{
    if (msg->left_timeout == true)
    {
        peri_left_state = 'T';
    }
    else
    {
        if (msg->left_inside == true)
        {
            peri_left_state = 'I';
        }
        else
        {
            peri_left_state = 'O';
        }
    }

    if (msg->right_timeout == true)
    {
        peri_right_state = 'T';
    }
    else
    {
        if (msg->right_inside == true)
        {
            peri_right_state = 'I';
        }
        else
        {
            peri_right_state = 'O';
        }
    }

    snprintf(line2, 15, "%c %i %c %i    ", peri_left_state, msg->left_mag, peri_right_state, msg->right_mag);
}

void ROSMower_oled::update()
{
    for (int i = 0; i < 3; i++)
    {

        oled_display_node::DisplayOutput output;
        output.actionType = 2;
        output.row = i;
        output.column = 1;
        output.numChars = 0;
        output.attributes = 0;
        switch (i)
        {
        case 0:
            output.text = line1;
            break;
        case 1:
            output.text = line2;
            break;
        case 2:
            output.text = line3;
            break;
        }

        pub_oled.publish(output);
    }
}
