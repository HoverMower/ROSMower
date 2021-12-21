
#ifndef CONFIG_H
#define CONFIG_H

/*------- Serial Interface -----*/
#define SERIAL_BAUDRATE 115200
#define SERIAL_RATE 50 // publish rate of messages in Hz

/*-------- DEBUG Section ----------*/
#define DEBUG_OUTPUT false       // Activate this for tests to get human readable data, DISABLE when using as ROS Node

/*------- Perimeter Definitions ------*/
#define pinPerimeterLeft A5        
#define pinPerimeterRight A4        
// #define pinLED 13               // Indicator if left is in/out
#define SWAP_COIL_POLARITY_LEFT false;
#define SWAP_COIL_POLARITY_RIGHT true;

/*----- user Button ---------*/
#define BUTTON true
#define pinButton 6

/*----- user Switches -------*/
#define pinSwitch1 2
#define pinSwitch2 3

/*----- Bumper -----------*/
#define BUMPER true
#define pinBumperLeft  4
#define pinBumperRight 5

/*----- BatteryMonitor -----------*/
#define BATMON true
#define pinBatterySwitch  13
#define pinBatteryVoltage A3
#define pinChargeCurrent A0
#define pinChargeVoltage A1
#define pinChargeEnable A2

/*------ Mow Motor --------- */
#define MOW true
#define pinMowCurrent A6
#define pinMowPWM 12
#define pinDirection 11
#define pinEnable 10
#define pinBreak 9
#define pinSpeed 8
#define pinAlarm 7

#endif
