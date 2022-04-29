#ifndef _ROSMOWER_SAFETYCONTROLLER_H
#define _ROSMOWER_SAFETYCONTROLLER_H

#include <ros/ros.h>
#include <string.h>
#include <stdio.h>
#include "std_msgs/Bool.h"
#include "rosmower_msgs/PerimeterMsg.h"
#include "rosmower_msgs/Bumper.h"
#include "rosmower_msgs/Battery.h"
#include "rosmower_msgs/MowMotor.h"
#include <dynamic_reconfigure/server.h>
#include <rosmower/SafetyControllerConfig.h>
#include <geometry_msgs/Twist.h>

class ROSMower_SafetyController
{
public:
    ROSMower_SafetyController();
    ~ROSMower_SafetyController();

    void batteryCallback(const rosmower_msgs::Battery::ConstPtr &msg);
    void perimeterCallback(const rosmower_msgs::PerimeterMsg::ConstPtr &msg);
    void bumperCallback(const rosmower_msgs::Bumper::ConstPtr &msg);
    void dyn_callback(ROSMower::SafetyControllerConfig &config, uint32_t level);
    void run();

private:
    void bumper_unstuck();
    void perimeter_unstuck();

    // Publishers
    ros::NodeHandle nh;
    ros::Publisher cmd_vel_bumper;
    ros::Publisher cmd_vel_perimeter;
    ros::Publisher pub_eStop;

    // Subscriber
    ros::Subscriber sub_battery;
    ros::Subscriber sub_perimeter;
    ros::Subscriber sub_bumper;

    // other
    bool allow_unstuck_bumper_ = false; // unstuck bumper active?
    int unstuck_bumper_attemps_ = 0;    // how often did we already tried to unstock?
    int unstuck_bumper_reverse_time_ = 0;
    double unstuck_bumper_reverse_speed_ = 0.0;
    int unstuck_bumper_rotate_time_ = 0;
    double unstuck_bumper_rotate_speed_ = 0.0;
    int max_bumper_attemps_ = 0; // how often should we try max before error?

    bool allow_unstuck_perimeter_ = false; // unstuck perimeter active?
    int unstuck_perimeter_attemps_ = 0;
    int unstuck_perimeter_reverse_time_ = 0;
    double unstuck_perimeter_reverse_speed_ = 0.0;
    int unstuck_perimeter_rotate_time_ = 0;
    double unstuck_perimeter_rotate_speed_ = 0.0;
    int max_perimeter_attemps_ = 0;   // how often should we try max before error?
    int blockingTimeout_unstuck_ = 0; // how long (s) to send 0 0 0 velocity, may needed for navigation to find new path

    // current states
    bool unstuck_bumper_ = false;   // do we actually try to unstuck?
    bool last_bumper_left_ = false; // last bumper event
    bool last_bumper_right_ = false;
    bool unstuck_bumper_left_ = false; // last bumper event
    bool unstuck_bumper_right_ = false;

    bool unstuck_perimeter_ = false;  //do we actually try to unstuck?
    bool last_peri_out_left_ = false; // last perimeter state
    bool last_peri_out_right_ = false;
    bool unstuck_perimeter_left_ = false; // last perimeter action
    bool unstuck_perimeter_right_ = false;

    // dynamic reconfigure
    typedef dynamic_reconfigure::Server<ROSMower::SafetyControllerConfig> DynamicReconfigServer;
    boost::shared_ptr<DynamicReconfigServer> param_reconfig_server_;
    DynamicReconfigServer::CallbackType param_reconfig_callback_;
};

#endif
