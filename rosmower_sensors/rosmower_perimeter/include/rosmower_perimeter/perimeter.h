#include <ros/ros.h>
#include "protocol.h"
#include "rosmower_msgs/PerimeterMsg.h"
#include "rosmower_msgs/Bumper.h"
#include "std_msgs/Int32.h"
#include <dynamic_reconfigure/server.h>
#include <rosmower_perimeter/PerimeterConfig.h>

class Perimeter
{
public:
    Perimeter();
    ~Perimeter();

    void read();
    void write();
    void dyn_callback(rosmower_perimeter::PerimeterConfig &config, uint32_t level);

private:
    void protocol_recv(unsigned char c);
    
    // Publishers
    ros::NodeHandle nh;
    ros::Publisher peri_pub;
    ros::Publisher button_pub;
    ros::Publisher bumper_pub;

    ros::Time last_read;

    // Perimeter protocol
    int port_fd;
    int msg_len = 0;
    unsigned char prev_byte = 0;
    uint16_t start_frame = 0;
    unsigned char *p;
    SerialFeedback msg;

    // additional attributes
    int peri_timeout_smag_; // timeout if smag below
    int peri_timeout_;      // timeout if one coil is not inside peri loop
    ros::Time lastTime_left_inside_;
    ros::Time lastTime_right_inside_;

    // dynamic reconfigure
    typedef dynamic_reconfigure::Server<rosmower_perimeter::PerimeterConfig> DynamicReconfigServer;
    boost::shared_ptr<DynamicReconfigServer> param_reconfig_server_;
    DynamicReconfigServer::CallbackType param_reconfig_callback_;
};