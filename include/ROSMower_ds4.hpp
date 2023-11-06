#ifndef _ROSMOWER_DS4_H
#define _ROSMOWER_DS4_H

#include <rclcpp/rclcpp.hpp>
#include <string.h>
#include <stdio.h>
#include "std_msgs/msg/bool.hpp"
#include "ds4_driver_msgs/msg/status.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "rosmower_msgs/srv/press_switch.hpp"
#include "rosmower_msgs/srv/set_switch.hpp"
#include "rosmower_msgs/srv/set_mow_motor.hpp"
#include "rosmower_msgs/msg/switches.hpp"
#include "rosmower_msgs/msg/mow_motor.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

class ROSMower_ds4 : public rclcpp::Node
{
public:
    ROSMower_ds4(std::string name);
    ~ROSMower_ds4();

    void ds4Callback(const std::shared_ptr<ds4_driver::msg::status> msg);
    void mowCallback(const std::shared_ptr<rosmower_msgs::msg::MowMotor> msg);
    void switchesCallback(const std::shared_ptr<rosmower_msgs::msg::Switches> msg);
    void update();

private:

    
    void publish_point();
    // Publishers
    rclcpp::Publisher<std_msgs::mgs::Bool>::SharedPtr pub_eStop;
    rclcpp::Publisher<geometry_msgs::mgs::PointStamped>::SharedPtr pub_point;
    rclcpp::Publisher<sensors_msgs::mgs::Joy>::SharedPtr pub_joy;

    // Service Clients
    rclcpp::Client<rosmower_msgs::msg::pressSwitch>::SharedPtr _srv_pressSwitch = this->create_client<rosmower_msgs::msg::pressSwitch>("hovermower/pressSwitch");
    rclcpp::Client<rosmower_msgs::msg::setSwitch>::SharedPtr _srv_setSwitch = this->create_client<rosmower_msgs::msg::setSwitch>("hovermower/setSwitch");
    rclcpp::Client<rosmower_msgs::msg::setMowMotor>::SharedPtr _srv_mow = this->create_client<rosmower_msgs::msg::setMowMotor>("hovermower/setMowMotorSpeed");

    // Subscriber
    rclcpp::Subscription<ds4_driver::msg::Status>::SharedPtr _sub_ds4;
    rclcpp::Subscription<rosmower_msgs::msg::MowMotor>::SharedPtr _sub_mow;
    rclcpp::Subscription<rosmower_msgs::msg::Switches>::SharedPtr _sub_switches;

    // tf
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener* tfListener;
    

    uint8_t _switch1;
    uint8_t _switch2;
    int _mow_speed;

    // keep last button state to not toggle too often (debounce)
    int _ds4_last_button_l1 = 0;
    int _ds4_last_button_l2 = 0;
    int _ds4_last_button_r1 = 0;
    int _ds4_last_button_circle = 0;
    int _ds4_last_button_square = 0;
    int _ds4_last_button_triangle = 0;
    int _ds4_last_button_cross = 0;

    bool _e_stop;
    bool _cross_pressed;

};

#endif
