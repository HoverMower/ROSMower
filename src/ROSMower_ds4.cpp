#include "ROSMower_ds4.h"

ROSMower_ds4::ROSMower_ds4()
{

    // Register  publisher
    pub_eStop = nh.advertise<std_msgs::Bool>("/e_stop", 3);

    // Register subscriber
    _sub_mow = nh.subscribe("hovermower/sensors/MowMotor", 1000, &ROSMower_ds4::mowCallback, this);
    _sub_switches = nh.subscribe("hovermower/switches", 10, &ROSMower_ds4::switchesCallback, this);
    _sub_ds4 = nh.subscribe("status", 10, &ROSMower_ds4::ds4Callback, this);

    // register service clients

    _e_stop = false;
}

ROSMower_ds4::~ROSMower_ds4()
{
}

void ROSMower_ds4::ds4Callback(const ds4_driver::Status::ConstPtr &msg)
{
    // e_stop
    // Ensures not to toggle for each incoming message, when button keeps pressed
    if (msg->button_circle > 0 && msg->button_circle != _ds4_last_button_circle)
    {
        _e_stop = !_e_stop;
        _ds4_last_button_circle = msg->button_circle;
    }
    // reset if button has been released
    if (msg->button_circle == 0)
    {
        _ds4_last_button_circle = 0;
    }

    // enable Hoverboard PCB
    if (msg->button_r1 > 0 && msg->button_r1 != _ds4_last_button_r1)
    {
        _ds4_last_button_r1 = msg->button_r1;
        rosmower_msgs::pressSwitch srv;
        srv.request.switch_id = 3;
        _srv_pressSwitch.call(srv);
    }
    // reset if button has been released
    if (msg->button_r1 == 0)
    {
        _ds4_last_button_r1 = 0;
    }    

    // LED headlights 1
    if (msg->button_l1 > 0 && msg->button_l1 != _ds4_last_button_l1)
    {
        _ds4_last_button_l1 = msg->button_l1;
        rosmower_msgs::setSwitch srv;
        srv.request.switch_id = 1;
        if (_switch1 > 0)
        {
            srv.request.value = 0;
        }
        else
        {
            srv.request.value = 100;
        }
        _srv_setSwitch.call(srv);
    }
    // reset if button has been released
    if (msg->button_l1 == 0)
    {
        _ds4_last_button_l1 = 0;
    }     
    // LED headlights 2
    if (msg->button_l2 > 0 && msg->button_l2 != _ds4_last_button_l2)
    {
        _ds4_last_button_l2 = msg->button_l2;
        rosmower_msgs::setSwitch srv;
        srv.request.switch_id = 2;
        if (_switch2 > 0)
        {
            srv.request.value = 0;
        }
        else
        {
            srv.request.value = 100;
        }
        _srv_setSwitch.call(srv);
    }
    // reset if button has been released
    if (msg->button_l2 == 0)
    {
        _ds4_last_button_l2 = 0;
    } 

    // mow motor speed
    if (msg->button_square > 0 && msg->button_square != _ds4_last_button_square)
    {
        _ds4_last_button_square = msg->button_square;
        rosmower_msgs::setMowMotor srv;
        if (_mow_speed == 0)
        {
            srv.request.Speed = 1000;
        }
        else
        {
            srv.request.Speed = 0;
        }
        _srv_mow.call(srv);
    }
    // reset if button has been released
    if (msg->button_square == 0)
    {
        _ds4_last_button_square = 0;
    }     
}

void ROSMower_ds4::mowCallback(const rosmower_msgs::MowMotor::ConstPtr &msg)
{
    _mow_speed = msg->speed;
}

void ROSMower_ds4::switchesCallback(const rosmower_msgs::Switches::ConstPtr &msg)
{
    _switch1 = msg->switch1;
    _switch2 = msg->switch2;
}

void ROSMower_ds4::update()
{
    std_msgs::Bool msg_estop;
    msg_estop.data = _e_stop;
    pub_eStop.publish(msg_estop);
}
