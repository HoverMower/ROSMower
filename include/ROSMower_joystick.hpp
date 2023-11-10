#ifndef _ROSMOWER_JOYSTICK_H
#define _ROSMOWER_JOYSTICK_H

#include <rclcpp/rclcpp.hpp>
#include <string.h>
#include <stdio.h>
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "rosmower_msgs/srv/press_switch.hpp"
#include "rosmower_msgs/srv/set_switch.hpp"
#include "rosmower_msgs/srv/set_mow_motor.hpp"
#include "rosmower_msgs/msg/switches.hpp"
#include "rosmower_msgs/msg/mow_motor.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class ROSMower_joystick : public rclcpp::Node
{
public:
    ROSMower_joystick(std::string name);
    ~ROSMower_joystick();

    void joyCallback(const std::shared_ptr<sensor_msgs::msg::Joy> msg);
    void mowCallback(const std::shared_ptr<rosmower_msgs::msg::MowMotor> msg);
    void switchesCallback(const std::shared_ptr<rosmower_msgs::msg::Switches> msg);

    enum buttons : int
    {
        square = 0,
        triangle = 1,
        star = 2,
        cross = 3,
        LB = 4,
        LT = 5,
        RB = 6,
        RT = 7,
        share = 8,
        options = 9,
        ten = 10,
        touch = 11,
        left_click = 12,
        right_click = 13,
        left = 14,
        up = 15,
        right = 16,
        down = 17
    };

private:
    void publish_point();
    // Publishers
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_eStop;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_point;

    // Service Clients
    rclcpp::Client<rosmower_msgs::srv::PressSwitch>::SharedPtr _srv_pressSwitch;// = this->create_client<rosmower_msgs::srv::PressSwitch>("hovermower/pressSwitch");
    rclcpp::Client<rosmower_msgs::srv::SetSwitch>::SharedPtr _srv_setSwitch;// = this->create_client<rosmower_msgs::srv::SetSwitch>("hovermower/setSwitch");
    rclcpp::Client<rosmower_msgs::srv::SetMowMotor>::SharedPtr _srv_mow;// = this->create_client<rosmower_msgs::srv::SetMowMotor>("hovermower/setMowMotorSpeed");

    // Subscriber
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _sub_joy;
    rclcpp::Subscription<rosmower_msgs::msg::MowMotor>::SharedPtr _sub_mow;
    rclcpp::Subscription<rosmower_msgs::msg::Switches>::SharedPtr _sub_switches;

    // tf
    // tf2_ros::Buffer tfBuffer;
    // tf2_ros::TransformListener *tfListener;

// Listener for the broadcast transform message
    std::shared_ptr<tf2_ros::TransformListener> tfListener_;
    // Buffer that stores several seconds of transforms for easy lookup by the listener
    std::shared_ptr<tf2_ros::Buffer> tfBuffer_;

    //tfBuffer_ = 
     //   std::make_unique<tf2_ros::Buffer>(this->get_clock());
    //tfListener_ =
     //   std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

    uint8_t _switch1;
    uint8_t _switch2;
    int _mow_speed;

    // keep last button state to not toggle too often (debounce)
    int last_button_l1 = 0;
    int last_button_l2 = 0;
    int last_button_r1 = 0;
    int last_button_circle = 0;
    int last_button_square = 0;
    int last_button_triangle = 0;
    int last_button_cross = 0;

    bool _e_stop;
    bool _cross_pressed;
};

#endif
