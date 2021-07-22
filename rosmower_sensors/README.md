# ROSMower Sensor package
This package contains firmware and ROS drivers for ROSMower special sensors, like perimeter receiver.

# Perimeter
This sensor can detect an perimeter wire (inside/outside) and supports two perimeter coils (left/right).
So robot is able to detect if it operates inside the defined area or not. Also it can detect if it leaves the area on left or right side.
This is used to steer into correct direction to go back to defined area.

The firmware is written for Arduino Nano connected to USB Port of robot main MCU (e.g. Raspberry PI). 

Perimeter sensor is based on the hard work of Ardumower team (www.ardumower.de). For details about used signal and firmware for perimeter sender, please have a look there.
