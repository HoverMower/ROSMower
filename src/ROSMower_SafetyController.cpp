#include "ROSMower_SafetyController.hpp"

ROSMower_SafetyController::ROSMower_SafetyController(std::string name) : Node(name)
{

    // Register  publisher
    pub_eStop = create_publisher<std_msgs::msg::Bool>("/e_stop", 3);
    sub_battery = create_subscription<rosmower_msgs::msg::Battery>("hovermower/sensors/Battery", 1000, std::bind(&ROSMower_SafetyController::batteryCallback, this, std::placeholders::_1));
    sub_perimeter = create_subscription<rosmower_msgs::msg::Perimeter>("hovermower/sensors/Perimeter", 1000, std::bind(&ROSMower_SafetyController::perimeterCallback, this, std::placeholders::_1));
    sub_bumper = create_subscription<rosmower_msgs::msg::Bumper>("hovermower/sensors/Bumper", 1000, std::bind(&ROSMower_SafetyController::bumperCallback, this, std::placeholders::_1));
    cmd_vel_bumper = create_publisher<geometry_msgs::msg::Twist>("/safety_bump_vel", 1000);

    // declare parameter
    declare_parameter("allow_unstuck_bumper", true);
    declare_parameter("unstuck_bumper_attempts", 3);
    declare_parameter("unstuck_bumper_reverse_time", 2000);
    declare_parameter("unstuck_bumper_reverse_speed", 0.5);
    declare_parameter("unstuck_bumper_rotate_time", 1000);
    declare_parameter("unstuck_bumper_rotate_speed", 1.0);
    declare_parameter("max_bumper_attempts", 3);

    declare_parameter("allow_unstuck_perimeter", true);
    declare_parameter("unstuck_perimeter_attempts", 3);
    declare_parameter("unstuck_perimeter_reverse_time", 2000);
    declare_parameter("unstuck_perimeter_reverse_speed", 0.5);
    declare_parameter("unstuck_perimeter_rotate_time", 1000);
    declare_parameter("unstuck_perimeter_rotate_speed", 1.0);
    declare_parameter("max_perimeter_attempts", 3);

    declare_parameter("blockingTimeout_unstuck", 1000);

    get_parameter("allow_unstuck_bumper", allow_unstuck_bumper_);
    get_parameter("unstuck_bumper_attempts", unstuck_bumper_attempts_);
    get_parameter("unstuck_bumper_reverse_time", unstuck_bumper_reverse_time_);
    get_parameter("unstuck_bumper_reverse_speed", unstuck_bumper_reverse_speed_);
    get_parameter("unstuck_bumper_rotate_time", unstuck_bumper_rotate_time_);
    get_parameter("unstuck_bumper_rotate_speed", unstuck_bumper_rotate_speed_);
    get_parameter("max_bumper_attempts", max_bumper_attempts_);


    get_parameter("allow_unstuck_perimeter", allow_unstuck_perimeter_);
    get_parameter("unstuck_perimeter_attempts", unstuck_perimeter_attempts_);
    get_parameter("unstuck_perimeter_reverse_time", unstuck_perimeter_reverse_time_);
    get_parameter("unstuck_perimeter_reverse_speed", unstuck_perimeter_reverse_speed_);
    get_parameter("unstuck_perimeter_rotate_time", unstuck_perimeter_rotate_time_);
    get_parameter("unstuck_perimeter_rotate_speed", unstuck_perimeter_rotate_speed_);
    get_parameter("max_perimeter_attempts", max_perimeter_attempts_);

    get_parameter("blockingTimeout_unstuck", blockingTimeout_unstuck_);
    
    // register parameter change callback handle
    callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&ROSMower_SafetyController::parametersCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "ROSMower_SafetyController initialized");
}

ROSMower_SafetyController::~ROSMower_SafetyController()
{
}

void ROSMower_SafetyController::run()
{
    if (allow_unstuck_bumper_)
        bumper_unstuck();

    if (allow_unstuck_perimeter_)
        perimeter_unstuck();
}

void ROSMower_SafetyController::batteryCallback(const rosmower_msgs::msg::Battery::SharedPtr msg)
{
}

void ROSMower_SafetyController::bumper_unstuck()
{
    geometry_msgs::msg::Twist cmd_vel_msg;
    rclcpp::Rate rate(50);

    if (unstuck_bumper_) // only if we need to unstuck
    {
        // is it still blocked?
        if (last_bumper_left_ == true || last_bumper_right_ == true)
        {

            // first try to reverse to unblock
            if (unstuck_bumper_attempts_ < max_bumper_attempts_)
            {
                RCLCPP_INFO(get_logger(), "ROSMower SafetyController, reverse to unstuck bumper");
                cmd_vel_msg.linear.x = unstuck_bumper_reverse_speed_ * -1;

                rclcpp::Time reverse_time = get_clock()->now() + rclcpp::Duration::from_seconds(unstuck_bumper_reverse_time_ / 1000.0);

                // reverse as long as defined in configuration
                while (rclcpp::ok() && reverse_time > get_clock()->now())
                {
                    cmd_vel_bumper->publish(cmd_vel_msg);
                   // rclcpp::spin_some(this);
                    rate.sleep();
                }
                unstuck_bumper_attempts_++;
            }
            // we raised the max attempts, EMERGENCY STOP
            else
            {
                RCLCPP_ERROR(get_logger(), "ROSMower SafetyController unable to unstuck bumper");
                cmd_vel_msg.linear.x = 0.0;
                cmd_vel_msg.angular.z = 0.0;
                cmd_vel_bumper->publish(cmd_vel_msg);

                std_msgs::msg::Bool msg_estop;
                msg_estop.data = true;
                pub_eStop->publish(msg_estop);
                //rclcpp::spin_some(this);
            }
        }
        else
        {
            // successfully unstuck
            // rotate from obstacle
            cmd_vel_msg.linear.x = 0;
            cmd_vel_msg.angular.z = unstuck_bumper_rotate_speed_; //  radian per second
            if (unstuck_bumper_left_ == true)
            {
                cmd_vel_msg.angular.z = cmd_vel_msg.angular.z * -1;
            }

            rclcpp::Time rotate_time = get_clock()->now() + rclcpp::Duration::from_seconds(unstuck_bumper_rotate_time_ / 1000.0);

            // reverse as long as defined in configuration
            while (rclcpp::ok() && rotate_time > get_clock()->now())
            {
                cmd_vel_bumper->publish(cmd_vel_msg);
               // rclcpp::spin_some(this);
                rate.sleep();
            }

            unstuck_bumper_attempts_ = 0;
            unstuck_bumper_ = false;
            unstuck_bumper_left_ = false;
            unstuck_bumper_right_ = false;
            RCLCPP_INFO(get_logger(), "ROSMower SafetyController unstuck bumper complete");
        }
    }
}
void ROSMower_SafetyController::perimeter_unstuck()
{
    geometry_msgs::msg::Twist cmd_vel_msg;
    rclcpp::Rate rate(50);

    if (unstuck_perimeter_) // only if we need to unstuck
    {
        // is it still outside?
        if (last_peri_out_left_ == true || last_peri_out_right_ == true)
        {

            // first try to reverse to unblock
            if (unstuck_perimeter_attempts_ < max_perimeter_attempts_)
            {
                RCLCPP_INFO(get_logger(), "ROSMower SafetyController, reverse to recover perimeter");
                cmd_vel_msg.linear.x = unstuck_perimeter_reverse_speed_ * -1;

                rclcpp::Time reverse_time = get_clock()->now() + rclcpp::Duration::from_seconds(unstuck_perimeter_reverse_time_ / 1000.0);

                // reverse as long as defined in configuration
                while (rclcpp::ok() && reverse_time > get_clock()->now())
                {
                    cmd_vel_perimeter->publish(cmd_vel_msg);
                    //rclcpp::spin_some(this);
                    rate.sleep();
                }
                unstuck_perimeter_attempts_++;
            }
            // we raised the max attempts, EMERGENCY STOP
            else
            {
                RCLCPP_ERROR(get_logger(), "ROSMower SafetyController unable to recover perimeter");
                cmd_vel_msg.linear.x = 0.0;
                cmd_vel_msg.angular.z = 0.0;
                cmd_vel_bumper->publish(cmd_vel_msg);

                std_msgs::msg::Bool msg_estop;
                msg_estop.data = true;
                pub_eStop->publish(msg_estop);
               // rclcpp::spin_some(this);
            }
        }
        else
        {
            // successfully unstuck
            // rotate from obstacle
            cmd_vel_msg.linear.x = 0;
            cmd_vel_msg.angular.z = unstuck_perimeter_rotate_speed_; //  radian per second
            if (unstuck_perimeter_left_ == true)
            {
                cmd_vel_msg.angular.z = cmd_vel_msg.angular.z * -1;
            }

            rclcpp::Time rotate_time = get_clock()->now() + rclcpp::Duration::from_seconds(unstuck_perimeter_rotate_time_ / 1000.0);

            // reverse as long as defined in configuration
            while (rclcpp::ok() && rotate_time > get_clock()->now())
            {
                cmd_vel_perimeter->publish(cmd_vel_msg);
               // rclcpp::spin_some(this);
                rate.sleep();
            }

            unstuck_perimeter_attempts_ = 0;
            unstuck_perimeter_ = false;
            unstuck_perimeter_left_ = false;
            unstuck_perimeter_right_ = false;
            RCLCPP_INFO(get_logger(), "ROSMower SafetyController recover perimeter complete");
        }
    }
}
void ROSMower_SafetyController::bumperCallback(const rosmower_msgs::msg::Bumper::SharedPtr msg)
{

    if (unstuck_bumper_ == false && (msg->left == true || msg->right == true))
    {
        unstuck_bumper_ = true;
        unstuck_bumper_left_ = msg->left;
        unstuck_bumper_right_ = msg->right;
        RCLCPP_WARN(get_logger(), "ROSMower SafetyController start unstuck bumper");
    }
    last_bumper_left_ = msg->left;
    last_bumper_right_ = msg->right;
}

void ROSMower_SafetyController::perimeterCallback(const rosmower_msgs::msg::Perimeter::SharedPtr msg)
{
    if (unstuck_perimeter_ == false && (msg->left_inside == false || msg->right_inside == false))
    {
        unstuck_perimeter_ = true;
        unstuck_perimeter_left_ = !msg->left_inside;
        unstuck_perimeter_right_ = !msg->right_inside;
        RCLCPP_WARN(get_logger(), "ROSMower SafetyController start recover perimeter");
    }
    last_peri_out_left_ = !msg->left_inside;
    last_peri_out_right_ = !msg->right_inside;
}

rcl_interfaces::msg::SetParametersResult ROSMower_SafetyController::parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    // Here update class attributes, do some actions, etc.
    for (const auto &param : parameters)
    { // TODO: format reconfigure nicely
        if (param.get_name() == "unstuck_perimeter")
        {
            allow_unstuck_perimeter_ = param.as_bool();
        }

        if (param.get_name() == "max_perimeter_attempts")
        {
            max_perimeter_attempts_ = param.as_int();
        }

        if (param.get_name() == "unstuck_perimeter_reverse_time")
        {
            unstuck_perimeter_reverse_time_ = param.as_int();
        }
        if (param.get_name() == "unstuck_perimeter_reverse_speed")
        {
            unstuck_perimeter_reverse_speed_ = param.as_double();
        }
        if (param.get_name() == "unstuck_perimeter_rotate_time")
        {
            unstuck_perimeter_rotate_time_ = param.as_int();
        }
        if (param.get_name() == "unstuck_perimeter_rotate_speed")
        {
            unstuck_perimeter_rotate_speed_ = param.as_double();
        }

        if (param.get_name() == "allow_unstuck_bumper")
        {
            allow_unstuck_bumper_ = param.as_bool();
        }
        if (param.get_name() == "unstuck_buper_reverse_time")
        {
            unstuck_bumper_reverse_time_ = param.as_int();
        }
        if (param.get_name() == "unstuck_bumper_reverse_speed")
        {
            unstuck_bumper_reverse_speed_ = param.as_double();
        }
        if (param.get_name() == "unstuck_bumper_rotate_time")
        {
            unstuck_bumper_rotate_time_ = param.as_int();
        }
        if (param.get_name() == "unstuck_bumper_reverse_speed")
        {
            unstuck_bumper_reverse_speed_ = param.as_double();
        }
        if (param.get_name() == "max_bumper_attempts")
        {
            max_bumper_attempts_ = param.as_int();
        }
        if (param.get_name() == "timeout_after_unstuck")
        {
            blockingTimeout_unstuck_ = param.as_int();
        }
    }
    RCLCPP_INFO(get_logger(), "ROSMower SafetyController Reconfigure Request: bumper_attempts: %i perimeter_attempts: %i suppress cmd_vel: %i sec bumper_reverse_time: %i ms",
                unstuck_bumper_attempts_,
                unstuck_perimeter_attempts_,
                blockingTimeout_unstuck_,
                unstuck_bumper_reverse_time_);
}
