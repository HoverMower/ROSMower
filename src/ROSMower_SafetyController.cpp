#include "ROSMower_SafetyController.h"

ROSMower_SafetyController::ROSMower_SafetyController()
{

    // Register  publisher
    pub_eStop = nh.advertise<std_msgs::Bool>("/e_stop", 3);
    sub_battery = nh.subscribe("hovermower/sensors/Battery", 1000, &ROSMower_SafetyController::batteryCallback, this);
    sub_perimeter = nh.subscribe("hovermower/sensors/Perimeter", 1000, &ROSMower_SafetyController::perimeterCallback, this);
    sub_bumper = nh.subscribe("hovermower/sensors/Bumper", 1000, &ROSMower_SafetyController::bumperCallback, this);
    cmd_vel_bumper = nh.advertise<geometry_msgs::Twist>("/safety_bump_vel", 1000);

    param_reconfig_callback_ = boost::bind(&ROSMower_SafetyController::dyn_callback, this, _1, _2);

    param_reconfig_server_.reset(new DynamicReconfigServer());
    param_reconfig_server_->setCallback(param_reconfig_callback_);

    ROS_INFO("ROSMower_SafetyController initialized");
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

void ROSMower_SafetyController::batteryCallback(const rosmower_msgs::Battery::ConstPtr &msg)
{
}

void ROSMower_SafetyController::bumper_unstuck()
{
    geometry_msgs::Twist cmd_vel_msg;
    ros::Rate rate(50);

    if (unstuck_bumper_) // only if we need to unstuck
    {
        // is it still blocked?
        if (last_bumper_left_ == true || last_bumper_right_ == true)
        {

            // first try to reverse to unblock
            if (unstuck_bumper_attemps_ < max_bumper_attemps_)
            {
                ROS_INFO("ROSMower SafetyController, reverse to unstuck bumper");
                cmd_vel_msg.linear.x = unstuck_bumper_reverse_speed_ * -1;

                ros::Time reverse_time = ros::Time::now() + ros::Duration(unstuck_bumper_reverse_time_ / 1000.0);

                // reverse as long as defined in configuration
                while (ros::ok() && reverse_time > ros::Time::now())
                {
                    cmd_vel_bumper.publish(cmd_vel_msg);
                    ros::spinOnce();
                    rate.sleep();
                }
                unstuck_bumper_attemps_++;
            }
            // we raised the max attemps, EMERGENCY STOP
            else
            {
                ROS_ERROR("ROSMower SafetyController unable to unstuck bumper");
                cmd_vel_msg.linear.x = 0.0;
                cmd_vel_msg.angular.z = 0.0;
                cmd_vel_bumper.publish(cmd_vel_msg);

                std_msgs::Bool msg_estop;
                msg_estop.data = true;
                pub_eStop.publish(msg_estop);
                ros::spinOnce();
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

            ros::Time rotate_time = ros::Time::now() + ros::Duration(unstuck_bumper_rotate_time_ / 1000.0);

            // reverse as long as defined in configuration
            while (ros::ok() && rotate_time > ros::Time::now())
            {
                cmd_vel_bumper.publish(cmd_vel_msg);
                ros::spinOnce();
                rate.sleep();
            }

            unstuck_bumper_attemps_ = 0;
            unstuck_bumper_ = false;
            unstuck_bumper_left_ = false;
            unstuck_bumper_right_ = false;
            ROS_INFO("ROSMower SafetyController unstuck bumper complete");
        }
    }
}
void ROSMower_SafetyController::perimeter_unstuck()
{
    geometry_msgs::Twist cmd_vel_msg;
    ros::Rate rate(50);

    if (unstuck_perimeter_) // only if we need to unstuck
    {
        // is it still outside?
        if (last_peri_out_left_ == true || last_peri_out_right_ == true)
        {

            // first try to reverse to unblock
            if (unstuck_perimeter_attemps_ < max_perimeter_attemps_)
            {
                ROS_INFO("ROSMower SafetyController, reverse to recover perimeter");
                cmd_vel_msg.linear.x = unstuck_perimeter_reverse_speed_ * -1;

                ros::Time reverse_time = ros::Time::now() + ros::Duration(unstuck_perimeter_reverse_time_ / 1000.0);

                // reverse as long as defined in configuration
                while (ros::ok() && reverse_time > ros::Time::now())
                {
                    cmd_vel_perimeter.publish(cmd_vel_msg);
                    ros::spinOnce();
                    rate.sleep();
                }
                unstuck_perimeter_attemps_++;
            }
            // we raised the max attemps, EMERGENCY STOP
            else
            {
                ROS_ERROR("ROSMower SafetyController unable to recover perimeter");
                cmd_vel_msg.linear.x = 0.0;
                cmd_vel_msg.angular.z = 0.0;
                cmd_vel_bumper.publish(cmd_vel_msg);

                std_msgs::Bool msg_estop;
                msg_estop.data = true;
                pub_eStop.publish(msg_estop);
                ros::spinOnce();
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

            ros::Time rotate_time = ros::Time::now() + ros::Duration(unstuck_perimeter_rotate_time_ / 1000.0);

            // reverse as long as defined in configuration
            while (ros::ok() && rotate_time > ros::Time::now())
            {
                cmd_vel_perimeter.publish(cmd_vel_msg);
                ros::spinOnce();
                rate.sleep();
            }

            unstuck_perimeter_attemps_ = 0;
            unstuck_perimeter_ = false;
            unstuck_perimeter_left_ = false;
            unstuck_perimeter_right_ = false;
            ROS_INFO("ROSMower SafetyController recover perimeter complete");
        }
    }
}
void ROSMower_SafetyController::bumperCallback(const rosmower_msgs::Bumper::ConstPtr &msg)
{

    if (unstuck_bumper_ == false && (msg->left == true || msg->right == true))
    {
        unstuck_bumper_ = true;
        unstuck_bumper_left_ = msg->left;
        unstuck_bumper_right_ = msg->right;
        ROS_WARN("ROSMower SafetyController start unstuck bumper");
    }
    last_bumper_left_ = msg->left;
    last_bumper_right_ = msg->right;
}

void ROSMower_SafetyController::perimeterCallback(const rosmower_msgs::PerimeterMsg::ConstPtr &msg)
{
    if (unstuck_perimeter_ == false && (msg->left_inside == false || msg->right_inside == false))
    {
        unstuck_perimeter_ = true;
        unstuck_perimeter_left_ = !msg->left_inside;
        unstuck_perimeter_right_ = !msg->right_inside;
        ROS_WARN("ROSMower SafetyController start recover perimeter");
    }
    last_peri_out_left_ = !msg->left_inside;
    last_peri_out_right_ = !msg->right_inside;
}

void ROSMower_SafetyController::dyn_callback(ROSMower::SafetyControllerConfig &config, uint32_t level)
{
    // TODO: format reconfigure nicely
    ROS_INFO("ROSMower SafetyController Reconfigure Request: bumper_attemps: %i perimeter_attemps: %i suppress cmd_vel: %i sec bumper_reverse_time: %i ms",
             config.bumper_unstuck_attemps,
             config.peri_unstuck_attemps,
             config.timeout_after_unstuck,
             config.bumper_reverse_time);

    allow_unstuck_perimeter_ = config.unstuck_perimeter;
    max_perimeter_attemps_ = config.peri_unstuck_attemps;
    unstuck_perimeter_reverse_time_ = config.peri_reverse_time;
    unstuck_perimeter_reverse_speed_ = config.peri_reverse_speed;
    unstuck_perimeter_rotate_time_ = config.peri_rotate_time;
    unstuck_perimeter_rotate_speed_ = config.peri_rotate_speed;

    allow_unstuck_bumper_ = config.unstuck_bumper;
    unstuck_bumper_reverse_time_ = config.bumper_reverse_time;
    unstuck_bumper_reverse_speed_ = config.bumper_reverse_speed;
    unstuck_bumper_rotate_time_ = config.bumper_rotate_time;
    unstuck_bumper_rotate_speed_ = config.bumper_rotate_speed;
    max_bumper_attemps_ = config.bumper_unstuck_attemps;
    blockingTimeout_unstuck_ = config.timeout_after_unstuck;
}
