#include "perimeter.h"
#include "config.h"
#include "protocol.h"

#include <fcntl.h>
#include <termios.h>

Perimeter::Perimeter(){

    // Register  publisher
    peri_pub = nh.advertise<rosmower_msgs::PerimeterMsg>("sensors/perimeter", 3);

    // Prepare serial port
      if ((port_fd = open(PORT, O_RDWR | O_NOCTTY | O_NDELAY)) < 0) {
        ROS_FATAL("Cannot open serial port to perimeter receiver");
        exit(-1);
    }
    
    // CONFIGURE THE UART -- connecting to the board
    // The flags (defined in /usr/include/termios.h - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html):
    struct termios options;
   /* tcgetattr(port_fd, &options);
    options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;		//<Set baud rate
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    tcflush(port_fd, TCIFLUSH);
    tcsetattr(port_fd, TCSANOW, &options);
*/
        tcgetattr(port_fd, &options);
cfmakeraw(&options);
options.c_cflag |= (CLOCAL | CREAD);
options.c_iflag &= ~(IXOFF | IXANY);

// set vtime, vmin, baud rate...
options.c_cc[VMIN] = 0;  // you likely don't want to change this
options.c_cc[VTIME] = 0; // or this

cfsetispeed(&options, B115200);
cfsetospeed(&options, B115200);

    tcflush(port_fd, TCIFLUSH);
    tcsetattr(port_fd, TCSANOW, &options);
}

Perimeter::~Perimeter() {
    if (port_fd != -1) 
        close(port_fd);
}

void Perimeter::read() {
    if (port_fd != -1) {
        unsigned char c;
        int i = 0, r = 0;

        while ((r = ::read(port_fd, &c, 1)) > 0 && i++ < 1024)
            protocol_recv(c);

        if (i > 0)
	        last_read = ros::Time::now();

        if (r < 0 && errno != EAGAIN)
            ROS_ERROR("Reading from serial %s failed: %d", PORT, r);
    }

    if ((ros::Time::now() - last_read).toSec() > 1) {
        ROS_FATAL("Timeout reading from serial %s failed", PORT);
    }
}

void Perimeter::protocol_recv (unsigned char byte) {
    start_frame = ((uint16_t)(byte) << 8) | prev_byte;

    // Read the start frame
    if (start_frame == START_FRAME && msg_len == 0) {
        p = (unsigned char*)&msg;
        *p++ = prev_byte;
        *p++ = byte;
        msg_len = 2;

    } else if (msg_len >= 2 && msg_len < sizeof(SerialFeedback)) {
        // Otherwise just read the message content until the end
        *p++ = byte;
        msg_len++;
    }

    if (msg_len == sizeof(SerialFeedback)) {
        
        uint16_t checksum = (uint16_t)(
            msg.start ^
            msg.left_mag ^
            msg.right_mag ^
            msg.left_smag ^
            msg.right_smag ^
	        msg.left_inside ^
	        msg.right_inside ^
            msg.left_timeout ^
            msg.right_timeout ^
            msg.calibrated); 

        if (msg.start == START_FRAME && msg.checksum == checksum) {
            rosmower_msgs::PerimeterMsg peri;

            peri.left_mag = msg.left_mag;
            peri.right_mag = msg.right_mag;
            peri.left_smag = msg.left_smag;
            peri.right_smag = msg.right_smag;
            peri.left_inside = msg.left_inside;
            peri.right_inside = msg.right_inside;
            peri.left_timeout = msg.left_timeout;
            peri.right_timeout = msg.right_timeout;
            peri.calibrated = msg.calibrated;

            peri_pub.publish(peri);

            
        } else {
            ROS_WARN("Perimeter checksum mismatch: %d vs %d", msg.checksum, checksum);
        }
        msg_len = 0;
    }
    prev_byte = byte;
}
