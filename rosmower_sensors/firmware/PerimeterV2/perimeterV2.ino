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

void setup()  {  
  Wire.begin();
  Serial.begin(SERIAL_BAUDRATE);  
  pinMode(pinLED, OUTPUT);

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
}


void printSerial(){
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
    Serial.println();   
    delay(500);
}

void sendMessage(){
  
  feedback.start = START_FRAME;
  feedback.left_mag = perimeter.getMagnitude(0);
  feedback.right_mag = perimeter.getMagnitude(1);
  feedback.left_smag = perimeter.getSmoothMagnitude(0);
  feedback.right_smag = perimeter.getSmoothMagnitude(1);
  feedback.left_inside = perimeter.isInside(0);
  feedback.right_inside = perimeter.isInside(1);
  feedback.left_timeout = perimeter.signalTimedOut(0);
  feedback.right_timeout = perimeter.signalTimedOut(1);
  feedback.calibrated = ADCMan.calibrationDataAvail();
  feedback.checksum = (uint16_t)(feedback.start ^ feedback.left_mag ^ feedback.right_mag ^ feedback.left_smag ^ feedback.right_smag ^ feedback.left_inside ^ feedback.right_inside ^ 
                                 feedback.left_timeout ^ feedback.right_timeout ^ feedback.calibrated );

  Serial.write((byte*)&feedback, sizeof(feedback));
}

void loop()  {     
      
  ADCMan.run();  
  
  if (Serial.available() > 0){
    char ch = (char)Serial.read();      
    if (ch == 'v') mode = !mode;
    if (ch == 'c') {
      Serial.println("calibrating ADC (power off sender for this!)...");
      Serial.flush();
      delay(5000);
      ADCMan.calibrate();
    }
  }
  
  if (millis() >= nextTime){
    nextTime = millis() + 1000 / SERIAL_RATE;
    if (perimeter.isInside(0) != inside){
      inside = perimeter.isInside(0);    
    }
    if (perimeter.isInside(0)) digitalWrite(pinLED, HIGH);    
      else digitalWrite(pinLED, LOW);    

      if (DEBUG_OUTPUT){
        printSerial();
      }
      else
      {
        sendMessage();
      }

  }
}
