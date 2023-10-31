#include "ROSMower_ds4.hpp"

ROSMower_ds4::ROSMower_ds4()
{

    // Register  publisher
    pub_eStop = create_publisher<std_msgs::msg::Bool>("/e_stop", 3);
    // pub_point = nh.advertise<geometry_msgs::PointStamped>("/clicked_point", 3);

    // Register subscriber
    _sub_mow = create_subscription<rosmower_msgs::msg::MowMotor>("hovermower/sensors/MowMotor", 1000, std::bind(&ROSMower_ds4::mowCallback, this,std::placeholder_1));
    _sub_switches = create_subscription<rosmower_msgs::msg::Switches>("hovermower/switches", 10, std::bind(&ROSMower_ds4::switchesCallback, this,std::placeholder_1));
    _sub_ds4 = create_subscription<ds4_driver::msg::Status>("status", 10, std::bind(&ROSMower_ds4::ds4Callback, this,std::placeholder_1));

    // register service clients

    _e_stop = false;

    tfListener = new tf2_ros::TransformListener(tfBuffer);
}

ROSMower_ds4::~ROSMower_ds4()
{
}

void ROSMower_ds4::ds4Callback(const std::shared_ptr<ds4_driver::msg::status> msg);
{
    // e_stop
    // Ensures not to toggle for each incoming message, when button keeps pressed
    if (msg->button_l1 > 0 && msg->button_l1 != _ds4_last_button_l1)
    {
        _e_stop = !_e_stop;
        _ds4_last_button_l1 = msg->button_l1;

        std_msgs::msg::Bool msg_estop;
        msg_estop.data = _e_stop;
        pub_eStop->publish(msg_estop);
       // ros::spinOnce();
    }
    // reset if button has been released
    if (msg->button_l1 == 0)
    {
        _ds4_last_button_l1 = 0;
    }

 
    // enable Hoverboard PCB
    if (msg->button_r1 > 0 && msg->button_r1 != _ds4_last_button_r1)
    {
        _ds4_last_button_r1 = msg->button_r1;
        auto srv = rosmower_msgs::msg::press_switch::Request();
        srv->switch_id = 3;
        _srv_pressSwitch->send_request(srv);
    }
    // reset if button has been released
    if (msg->button_r1 == 0)
    {
        _ds4_last_button_r1 = 0;
    }


  
    // mow motor speed
    if (msg->button_l2 > 0 && msg->button_l2 != _ds4_last_button_l2)
    {
        _ds4_last_button_l2 = msg->button_l2;
        auto srv = rosmower_msgs::set_mow_motor::Request();
        if (_mow_speed == 0)
        {
            srv->Speed = 1500;
        }
        else
        {
            srv->Speed = 0;
        }
        _srv_mow.send_request(srv);
    }
    // reset if button has been released
    if (msg->button_l2 == 0)
    {
        _ds4_last_button_l2 = 0;
    }
}

void ROSMower_ds4::mowCallback(const std::shared_ptr<rosmower_msgs::msg::MowMotor> msg)
{
    _mow_speed = msg->speed;
}

void ROSMower_ds4::switchesCallback(const std::shared_ptr<rosmower_msgs::msg::Switches> msg)
{
    _switch1 = msg->switch1;
    _switch2 = msg->switch2;
}

void ROSMower_ds4::publish_point()
{
    geometry_msgs::TransformStamped transformStamped;
    try
    {
        transformStamped = tfBuffer.lookupTransform("map", "base_link",
                                                    rclcpp::Time(0), rclcpp::Duration(5.0));
        geometry_msgs::msg::PointStamped point;
        point->header.stamp = get_clock();
        point->header.frame_id = "map";
        point->point.x = transformStamped.transform.translation.x;
        point->point.y = transformStamped.transform.translation.y;
        point->point.z = 0;
        pub_point.publish(point);
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_WARN(get_logger(),"%s", ex.what());
    }
}
