
#ifndef CONFIG_H
#define CONFIG_H

/*------- Serial Interface -----*/
#define SERIAL_BAUDRATE 115200
#define SERIAL_RATE 50 // publish rate of messages in Hz

/*------- PIN Definitions ------*/
#define pinPerimeterLeft A5        
#define pinPerimeterRight A4        
#define pinLED 13               // Indicator if left is in/out

/*-------- DEBUG Section ----------*/
#define DEBUG_OUTPUT false       // Activate this for tests to get human readable data, DISABLE when using as ROS Node


/*------ Perimeter Coils ------------*/
#define SWAP_COIL_POLARITY_LEFT false;
#define SWAP_COIL_POLARITY_RIGHT true;

/*-----other sensors active?---------*/
#define BUTTON true
#define pinButton 3

#define BUMPER true
#define pinBumperLeft  4
#define pinBumperRight 5

#endif
