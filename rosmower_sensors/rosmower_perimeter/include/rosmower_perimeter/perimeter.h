#include <ros/ros.h>
#include "protocol.h"
#include "rosmower_msgs/PerimeterMsg.h"

class Perimeter{
public:
    Perimeter();
    ~Perimeter();

    void read();
    void write();

private:

    void protocol_recv (unsigned char c);
    // Publishers
    ros::NodeHandle nh;
    ros::Publisher peri_pub;

    ros::Time last_read;

    // Perimeter protocol
    int port_fd;
    int msg_len = 0;
    unsigned char prev_byte = 0;
    uint16_t start_frame = 0;
    unsigned char* p;
    SerialFeedback msg;

};