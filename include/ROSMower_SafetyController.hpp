#ifndef _ROSMOWER_SAFETYCONTROLLER_H
#define _ROSMOWER_SAFETYCONTROLLER_H

#include <rclcpp/rclcpp.hpp>
#include <string.h>
#include <stdio.h>
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "rosmower_msgs/msg/perimeter.hpp"
#include "rosmower_msgs/msg/mow_motor.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include "rcl_interfaces/msg/set_parameters_result.hpp"

class ROSMower_SafetyController : public rclcpp::Node
{
public:
    ROSMower_SafetyController(std::string name);
    ~ROSMower_SafetyController();

    void batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg);
    void perimeterCallback(const rosmower_msgs::msg::Perimeter::SharedPtr msg);
    void bumperleftCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void bumperrightCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void run();
    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter> &parameters);

private:
    void bumper_unstuck();
    void perimeter_unstuck();

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_bumper;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_perimeter;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_eStop;

    // Subscriber
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr sub_battery;
    rclcpp::Subscription<rosmower_msgs::msg::Perimeter>::SharedPtr sub_perimeter;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_bumper_left;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_bumper_right;

    // Parameter Callback handle
    OnSetParametersCallbackHandle::SharedPtr callback_handle_;

    // other
    bool allow_unstuck_bumper_ = false; // unstuck bumper active?
    int unstuck_bumper_attempts_ = 0;    // how often did we already tried to unstock?
    int unstuck_bumper_reverse_time_ = 0;
    double unstuck_bumper_reverse_speed_ = 0.0;
    int unstuck_bumper_rotate_time_ = 0;
    double unstuck_bumper_rotate_speed_ = 0.0;
    int max_bumper_attempts_ = 0; // how often should we try max before error?

    bool allow_unstuck_perimeter_ = false; // unstuck perimeter active?
    int unstuck_perimeter_attempts_ = 0;
    int unstuck_perimeter_reverse_time_ = 0;
    double unstuck_perimeter_reverse_speed_ = 0.0;
    int unstuck_perimeter_rotate_time_ = 0;
    double unstuck_perimeter_rotate_speed_ = 0.0;
    int max_perimeter_attempts_ = 0;   // how often should we try max before error?
    int blockingTimeout_unstuck_ = 0; // how long (s) to send 0 0 0 velocity, may needed for navigation to find new path

    // current states
    bool unstuck_bumper_ = false;   // do we actually try to unstuck?
    bool last_bumper_left_ = false; // last bumper event
    bool last_bumper_right_ = false;
    bool unstuck_bumper_left_ = false; // last bumper event
    bool unstuck_bumper_right_ = false;

    bool unstuck_perimeter_ = false;  // do we actually try to unstuck?
    bool last_peri_out_left_ = false; // last perimeter state
    bool last_peri_out_right_ = false;
    bool unstuck_perimeter_left_ = false; // last perimeter action
    bool unstuck_perimeter_right_ = false;
};

#endif
