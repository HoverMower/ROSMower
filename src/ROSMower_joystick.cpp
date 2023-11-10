#include "ROSMower_joystick.hpp"

ROSMower_joystick::ROSMower_joystick(std::string name) : Node(name)
{

    // Register  publisher
    pub_eStop = create_publisher<std_msgs::msg::Bool>("/e_stop", 3);
    // pub_point = nh.advertise<geometry_msgs::PointStamped>("/clicked_point", 3);

    // Register subscriber
    _sub_mow = create_subscription<rosmower_msgs::msg::MowMotor>("hovermower/sensors/MowMotor", 1000, std::bind(&ROSMower_joystick::mowCallback, this, std::placeholders::_1));
    _sub_switches = create_subscription<rosmower_msgs::msg::Switches>("hovermower/switches", 10, std::bind(&ROSMower_joystick::switchesCallback, this, std::placeholders::_1));
    _sub_joy = create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&ROSMower_joystick::joyCallback, this, std::placeholders::_1));

    // register service clients
    _srv_pressSwitch = create_client<rosmower_msgs::srv::PressSwitch>("hovermower/pressSwitch");
    _srv_setSwitch = this->create_client<rosmower_msgs::srv::SetSwitch>("hovermower/setSwitch");
    _srv_mow = this->create_client<rosmower_msgs::srv::SetMowMotor>("hovermower/setMowMotorSpeed");

    _e_stop = false;

    // tfListener = new tf2_ros::TransformListener(tfBuffer);
    tfBuffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    // Listen to the buffer of transforms
    tfListener_ = std::make_unique<tf2_ros::TransformListener>(*tfBuffer_);
}

ROSMower_joystick::~ROSMower_joystick()
{
}

void ROSMower_joystick::joyCallback(const std::shared_ptr<sensor_msgs::msg::Joy> msg)
{
    // e_stop
    // Ensures not to toggle for each incoming message, when button keeps pressed
    if (msg->buttons[buttons::LB] > 0 && msg->buttons[buttons::LB] != last_button_l1)
    {
        _e_stop = !_e_stop;
        last_button_l1 = msg->buttons[buttons::LB];

        std_msgs::msg::Bool msg_estop;
        msg_estop.data = _e_stop;
        pub_eStop->publish(msg_estop);
        // ros::spinOnce();
    }
    // reset if button has been released
    if (msg->buttons[buttons::LB] == 0)
    {
        last_button_l1 = 0;
    }

    // enable Hoverboard PCB
    if (msg->buttons[buttons::RB] > 0 && msg->buttons[buttons::RB] != last_button_r1)
    {
        last_button_r1 = msg->buttons[buttons::RB];
        auto srv = std::make_shared<rosmower_msgs::srv::PressSwitch::Request>();
        srv->switch_id = 3;
        _srv_pressSwitch->async_send_request(srv);
    }
    // reset if button has been released
    if (msg->buttons[buttons::RB] == 0)
    {
        last_button_r1 = 0;
    }

    // mow motor speed
    if (msg->buttons[buttons::LT] > 0 && msg->buttons[buttons::LT] != last_button_l2)
    {
        last_button_l2 = msg->buttons[buttons::LT];
        auto srv = std::make_shared<rosmower_msgs::srv::SetMowMotor::Request>();
        if (_mow_speed == 0)
        {
            srv->speed = 1500;
        }
        else
        {
            srv->speed = 0;
        }
        _srv_mow->async_send_request(srv);
    }
    // reset if button has been released
    if (msg->buttons[buttons::LT] == 0)
    {
        last_button_l2 = 0;
    }
}

void ROSMower_joystick::mowCallback(const std::shared_ptr<rosmower_msgs::msg::MowMotor> msg)
{
    _mow_speed = msg->speed;
}

void ROSMower_joystick::switchesCallback(const std::shared_ptr<rosmower_msgs::msg::Switches> msg)
{
    _switch1 = msg->switch1;
    _switch2 = msg->switch2;
}

void ROSMower_joystick::publish_point()
{
    geometry_msgs::msg::TransformStamped transformStamped;
    try
    {
        transformStamped = tfBuffer_->lookupTransform("map", "base_link",
                                                      rclcpp::Time(0), rclcpp::Duration::from_seconds(5.0));
        geometry_msgs::msg::PointStamped point;
        point.header.stamp = get_clock()->now();
        point.header.frame_id = "map";
        point.point.x = transformStamped.transform.translation.x;
        point.point.y = transformStamped.transform.translation.y;
        point.point.z = 0;
        pub_point->publish(point);
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_WARN(get_logger(), "%s", ex.what());
    }
}
