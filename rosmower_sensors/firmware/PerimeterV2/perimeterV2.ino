/*

  Firmware for Standalone Perimeter Receiver based on Ardumower Receiver
  This firmware supports two perimeter coils (left/right) which are attached to Arduino Nano

  Firmware reads perimeter values and reports data via USB to an attached device (like Raspberry PI running ROS)

  Private-use only! (you need to ask for a commercial-use)

  The code is open: you can modify it under the terms of the
  GNU General Public License as published by the Free Software Foundation,
  either version 3 of the License, or (at your option) any later version.

  The code is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

  Private-use only! (you need to ask for a commercial-use)


*/

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>

#include "drivers.h"
#include "adcman.h"
#include "perimeter.h"
#include "config.h"
#include "protocol.h"

Perimeter perimeter;
unsigned long nextTime = 0;
int counter = 0;
boolean inside = true;
int mode = 0;
static SerialFeedback feedback;

// Button
byte buttonCounter;
byte lastButtonCount;
unsigned long nextTimeButtonCheck;
unsigned long nextTimeButton;
// Bumper sensor
boolean bumperLeft;
boolean bumperRight;
unsigned long nextTimeBumper;

// Battery
unsigned long nextTimeBatteryCheck;
float batVoltage;
float batSwitchOffIfBelow;     // switch off if below voltage (Volt)
int batSwitchOffIfIdle;        // switch off battery if idle for minutes
float batFactor;               // battery conversion factor
float batChgFactor;            // battery conversion factor
float batFull;                 // battery reference Voltage (fully charged)
float batChargingCurrentMax;   // maximum current your charger can devliver
float batFullCurrent;          // current flowing when battery is fully charged
float startChargingIfBelow;    // start charging if battery Voltage is below
float chgFactor       ;     // charge current conversion factor
float chgVoltage ;  // charge voltage (Volt)
float chgCurrent ;  // charge current  (Ampere)
unsigned long chargingTimeout; // safety timer for charging

void setup()
{
  Wire.begin();
  Serial.begin(SERIAL_BAUDRATE);
  //  pinMode(pinLED, OUTPUT);

  delay(100);
  Serial.println("START");

  ADCMan.init();
  perimeter.setPins(pinPerimeterLeft, pinPerimeterRight);
  perimeter.useDifferentialPerimeterSignal = true;

  perimeter.speedTest();

  Serial.println("press...");
  Serial.println("  v to toggle between serial chart/Serial output");
  Serial.println("  c to calibrate zero point (sender must be off!)");
  delay(1000);

  // button
  pinMode(pinButton, INPUT);
  pinMode(pinButton, INPUT_PULLUP);

  pinMode(pinBumperLeft, INPUT);
  pinMode(pinBumperLeft, INPUT_PULLUP);
  pinMode(pinBumperRight, INPUT);
  pinMode(pinBumperRight, INPUT_PULLUP);

  // battery
  pinMode(pinBatteryVoltage, INPUT);        
  pinMode(pinChargeCurrent, INPUT);          
  pinMode(pinChargeVoltage, INPUT);            
  pinMode(pinChargeEnable, OUTPUT);


  buttonCounter = 0;
  lastButtonCount = 0;
  bumperLeft = false;
  bumperRight = false;
  nextTimeButtonCheck = millis() + 50;
  nextTimeBumper = millis() + 50;
  nextTimeBatteryCheck = millis() + 50;

  batFactor = voltageDividerUges(150, 10, 1.0) * ADC2voltage(1) * 10; // ADC to battery voltage factor *10
}

void printSerial()
{
  Serial.print("mag ");
  Serial.print((int)perimeter.getMagnitude(0));
  Serial.print(",");
  Serial.print((int)perimeter.getMagnitude(1));
  Serial.print("\t");
  Serial.print("\t");
  Serial.print("smag ");
  Serial.print((int)perimeter.getSmoothMagnitude(0));
  Serial.print(",");
  Serial.print((int)perimeter.getSmoothMagnitude(1));
  Serial.print("\t");
  Serial.print("in ");
  Serial.print((int)perimeter.isInside(0));
  Serial.print(",");
  Serial.print((int)perimeter.isInside(1));
  Serial.print("\t");
  Serial.print("on ");
  Serial.print((int)(!perimeter.signalTimedOut(0)));
  Serial.print(",");
  Serial.print((int)(!perimeter.signalTimedOut(1)));
  Serial.print("\t");
  Serial.print("adc ");
  Serial.print((int)(ADCMan.getCapturedChannels()));
  Serial.print("\t");
  Serial.print("bumL");
  Serial.print((int)bumperLeft);
  Serial.print("\t");
  Serial.print("bumR");
  Serial.print((int)bumperRight);
  Serial.print("\t");
  Serial.print("btn ");
  Serial.print((int)lastButtonCount);
  Serial.println();
}

void sendMessage()
{

  feedback.start = (uint16_t)START_FRAME;
  feedback.left_mag = (int16_t)perimeter.getMagnitude(0);
  feedback.right_mag = (int16_t)perimeter.getMagnitude(1);
  feedback.left_smag = (int16_t)perimeter.getSmoothMagnitude(0);
  feedback.right_smag = (int16_t)perimeter.getSmoothMagnitude(1);
  feedback.left_inside = (bool)perimeter.isInside(0);
  feedback.right_inside = (bool)perimeter.isInside(1);
  feedback.left_timeout = (bool)perimeter.signalTimedOut(0);
  feedback.right_timeout = (bool)perimeter.signalTimedOut(1);
  feedback.calibrated = (bool)ADCMan.calibrationDataAvail();
  feedback.bumperLeft = bumperLeft;
  feedback.bumperRight = bumperRight;
  feedback.buttonCount = lastButtonCount;
  feedback.checksum = (uint16_t)(feedback.start ^ feedback.left_mag ^ feedback.right_mag ^ feedback.left_smag ^ feedback.right_smag ^ feedback.left_inside ^ feedback.right_inside ^
                                 feedback.left_timeout ^ feedback.right_timeout ^ feedback.calibrated ^ feedback.bumperLeft ^ feedback.bumperRight ^ feedback.buttonCount);

  Serial.write((uint8_t *)&feedback, sizeof(feedback));
}

void loop()
{

  ADCMan.run();

  if (Serial.available() > 0)
  {
    char ch = (char)Serial.read();
    if (ch == 'v')
      mode = !mode;
    if (ch == 'c')
    {
      Serial.println("calibrating ADC (power off sender for this!)...");
      Serial.flush();
      delay(5000);
      ADCMan.calibrate();
    }
  }

  if (BUTTON)
    checkButton();
  if (BUMPER)
    checkBumper();

  /* Set info to ROS */
  if (millis() >= nextTime)
  {
    nextTime = millis() + 1000 / SERIAL_RATE;
    if (DEBUG_OUTPUT)
    {
      printSerial();
    }
    else
    {
      sendMessage();
    }
    lastButtonCount = 0;
  }
}

void checkButton()
{
  if ((millis() < nextTimeButtonCheck))
    return;

  nextTimeButtonCheck = millis() + 50;
  boolean buttonPressed = (digitalRead(pinButton) == LOW);

  if (((!buttonPressed) && (buttonCounter > 0)) || ((buttonPressed) && (millis() >= nextTimeButton)))
  {
    nextTimeButton = millis() + 1000;
    if (buttonPressed)
    {
      buttonCounter++;
    }
    else
    {
      lastButtonCount = buttonCounter;
      // Button has been released, buttonCounter has correct value now
      buttonCounter = 0;
    }
  }
}

void checkBumper()
{
  if ((millis() >= nextTimeBumper))
  {
    nextTimeBumper = millis() + 50;

    bumperLeft = (digitalRead(pinBumperLeft) == HIGH);
    bumperRight = (digitalRead(pinBumperRight) == HIGH);
  }
}

void checkBattery()
{
  if (BATMON)
  {
    if (millis() < nextTimeBatteryCheck)
      return;
    nextTimeBatteryCheck = millis() + 1000;

    // convert to double  
    int batADC = ADCMan.read(pinBatteryVoltage);
		int currentADC = ADCMan.read(pinChargeCurrent);
		int chgADC = ADCMan.read(pinChargeVoltage);    
		
    double batvolt = ((double)batADC) * batFactor;    
    double chgvolt = ((double)chgADC) * batChgFactor; 
		double curramp = ((double)currentADC) * chgFactor;

    // low-pass filter
    double accel = 0.01;
		//double accel = 1.0;
    if (abs(batVoltage-batvolt)>5)   batVoltage = batvolt; else batVoltage = (1.0-accel) * batVoltage + accel * batvolt;
    if (abs(chgVoltage-chgvolt)>5)   chgVoltage = chgvolt; else chgVoltage = (1.0-accel) * chgVoltage + accel * chgvolt;
		if (abs(chgCurrent-curramp)>0.5) chgCurrent = curramp; else chgCurrent = (1.0-accel) * chgCurrent + accel * curramp;       

    
  }
}

// Spannungsteiler Gesamtspannung ermitteln (Reihenschaltung R1-R2, U2 bekannt, U_GES zu ermitteln)
float voltageDividerUges(float R1, float R2, float U2)
{
  return (U2 / R2 * (R1 + R2)); // Uges
}

// ADC-value to voltage
float ADC2voltage(float ADCvalue)
{
  return (ADCvalue / 1023.0 * IOREF); // ADCman works @ 10 bit
}
