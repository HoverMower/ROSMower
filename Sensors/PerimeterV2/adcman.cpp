/*
  Ardumower (www.ardumower.de)
  Copyright (c) 2013-2014 by Alexander Grau
  Copyright (c) 2013-2014 by Sven Gennat
  Copyright (c) 2014 by Maxime Carpentieri
  
  Private-use only! (you need to ask for a commercial-use) 
 
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
  
  Private-use only! (you need to ask for a commercial-use)
*/

#ifndef __AVR__
  #include <chip.h>
#endif
#include <Arduino.h>
#include <limits.h>
#include "adcman.h"
#include "drivers.h"

#define ADDR 500
#define MAGIC 2

#ifdef __AVR__
  #define CHANNELS 16
#else
  #define CHANNELS 12
#endif 

#define NO_CHANNEL 255

volatile short position = 0;
volatile int16_t lastvalue = 0;
volatile uint8_t channel = 0;
volatile boolean busy = false;
int8_t *capture[CHANNELS]; // ADC capture buffer (ADC0-ADC7) - 8 bit signed (signed: zero = ADC/2)     
uint8_t captureSize[CHANNELS]; // ADC sample buffer size (ADC0-ADC7)
int16_t ofs[CHANNELS]; // ADC zero offset (ADC0-ADC7)
int16_t ADCMin[CHANNELS]; // ADC min sample value (ADC-ADC7)
int16_t ADCMax[CHANNELS]; // ADC max sample value (ADC-ADC7)
int16_t ADCAvg[CHANNELS]; // ADC avg sample value (ADC-ADC7)
volatile boolean captureComplete[CHANNELS]; // ADC buffer filled?
boolean autoCalibrate[CHANNELS]; // do auto-calibrate? (ADC0-ADC7)
  
ADCManager ADCMan;


ADCManager::ADCManager(){
  calibrationAvail = false;
  for (int i=0; i < CHANNELS; i++) {
    captureSize[i]=0;
    ofs[i]=0;
    captureComplete[i]=false;
    capture[i] = NULL;
    autoCalibrate[i] = false;
    ADCMax[i] = 0;
    ADCMin[i] = 0;
    ADCAvg[i] = 0;    
  }
  capturedChannels = 0;  
  //sampleRate = SRATE_19231;
  sampleRate = SRATE_38462;
  //sampleRate = SRATE_9615;
}

void ADCManager::init(){    
#ifndef __AVR__
  // free running ADC mode, f = ( adclock / 21 cycles per conversion )
  // example f = 19231  Hz:  using ADCCLK=405797 will result in a adcclock=403846 (due to adc_init internal conversion)
  uint32_t adcclk;
  switch (sampleRate){
    case SRATE_38462: adcclk = 811595; break;
    case SRATE_19231: adcclk = 405797; break;
    case SRATE_9615 : adcclk = 202898; break;
  }  
  adc_init(ADC, SystemCoreClock, adcclk, ADC_STARTUP_FAST); // startup=768 clocks
  adc_configure_timing(ADC, 0, ADC_SETTLING_TIME_3, 1);  // tracking=0, settling=17, transfer=1    
  ADC->ADC_MR |= ADC_MR_FREERUN_ON;   // free running  
  NVIC_EnableIRQ(ADC_IRQn);    
#endif
  delay(500); // wait for ADCRef to settle (stable ADCRef required for later calibration)
  if (loadCalib()) printCalib();
}

void ADCManager::setCapture(byte pin, byte samplecount, boolean autoCalibrateOfs){
  int ch = pin-A0;
  captureSize[ch] = samplecount;
  capture[ch] = new int8_t[samplecount];    
  autoCalibrate[ch] = autoCalibrateOfs;
}

void ADCManager::calibrate(){
  Serial.println("ADC calibration...");
  for (int ch=0; ch < CHANNELS; ch++){        
    ofs[ch] = 0;    
    if (autoCalibrate[ch]){
      calibrateOfs(A0 + ch);
    }
  }
  printCalib();  
  saveCalib();
  calibrationAvail = true;
}

boolean ADCManager::calibrationDataAvail(){
  return calibrationAvail;
}

void ADCManager::calibrateOfs(byte pin){  
  int ch = pin-A0;
  ofs[ch]=0;    
  for (int i=0; i < 10; i++){
    captureComplete[ch]=false;      
    while (!isCaptureComplete(pin)) {
      delay(20);
      run();    
    } 
  }
  int16_t center = ADCMin[ch] + (ADCMax[ch] - ADCMin[ch]) / 2.0;
  ofs[ch] = center;   

  Serial.print(F("ADC calibration ch"));
  Serial.print(ch);
  Serial.print(F("="));
  Serial.println(ofs[ch]);
}

void ADCManager::printCalib(){
  Serial.println(F("---ADC calib---"));  
  Serial.print(F("ADC sampleRate="));
  switch (sampleRate){
    case SRATE_38462: Serial.println(F("38462")); break;
    case SRATE_19231: Serial.println(F("19231")); break;
    case SRATE_9615 : Serial.println(F("9615")); break;
  }    
  for (int ch=0; ch < CHANNELS; ch++){
    Serial.print(F("AD"));
    Serial.print(ch);
    Serial.print(F("\t"));    
    Serial.print(F("min="));    
    Serial.print(ADCMin[ch]);
    Serial.print(F("\t"));    
    Serial.print(F("max="));    
    Serial.print(ADCMax[ch]);    
    Serial.print(F("\t"));    
    Serial.print(F("diff="));        
    Serial.print(ADCMax[ch]-ADCMin[ch]);    
    Serial.print(F("\t"));    
    Serial.print(F("ofs="));    
    Serial.println(ofs[ch]);
  }
}

void ADCManager::startADC(int sampleCount){  
//  Serial.print("startADC ch");
//  Serial.println(channel);
#ifdef __AVR__
  // http://www.atmel.com/images/doc2549.pdf
  /*  REFS0 : VCC use as a ref, IR_AUDIO : channel selection, ADEN : ADC Enable, ADSC : ADC Start, ADATE : ADC Auto Trigger Enable, ADIE : ADC Interrupt Enable,  ADPS : ADC Prescaler  */
  // free running ADC mode, f = ( 16MHz / prescaler ) / 13 cycles per conversion   
  ADMUX = _BV(REFS0) | (channel & 0x07); // | _BV(ADLAR); 
#if defined(__AVR_ATmega2560__)  
  ADCSRB = (ADCSRB & ~(1 << MUX5)) | (((channel >> 3) & 0x01) << MUX5);  
#endif  
  // use slow but accurate sampling if one sample only
  if (sampleCount == 1)
    // slow but accurate
    ADCSRA = _BV(ADSC) | _BV(ADEN) | _BV(ADATE) | _BV(ADIE) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0); // prescaler 128 : 9615 Hz      
  else {   
    switch (sampleRate){
      case SRATE_38462: ADCSRA = _BV(ADSC) | _BV(ADEN) | _BV(ADATE) | _BV(ADIE) | _BV(ADPS2) | _BV(ADPS0); break; // prescaler 32 : 38462 Hz        
      case SRATE_19231: ADCSRA = _BV(ADSC) | _BV(ADEN) | _BV(ADATE) | _BV(ADIE) | _BV(ADPS2) | _BV(ADPS1); break; //prescaler 64 : 19231 Hz   
      case SRATE_9615 : ADCSRA = _BV(ADSC) | _BV(ADEN) | _BV(ADATE) | _BV(ADIE) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0); break; // prescaler 128 : 9615 Hz        
    }        
  }
  // disable digital buffers (reduces noise/capacity)
  if (channel < 8) DIDR0 |= (1 << channel);
#if defined(__AVR_ATmega2560__)  
    else DIDR2 |= (1 << (channel-8));
#endif    
//  sei();   
#else 
  adc_enable_channel( ADC, (adc_channel_num_t)g_APinDescription[A0+channel].ulADCChannelNumber  ); 
  adc_enable_interrupt(ADC, ADC_IER_DRDY);  
  adc_start( ADC );  
#endif      
}
  
void ADCManager::startCapture(int sampleCount){
  //Serial.print("starting capture ch");
  //Serial.println(channel);
  position = 0;
  busy=true;
  startADC(sampleCount);  
}


ISR(ADC_vect){
  volatile int16_t value = ADC;    

  if (!busy) return;
  if (position >= captureSize[channel]){    
    busy=false;
    return;
  } 
  value -= ofs[channel];                   
  capture[channel][position] =  min(SCHAR_MAX,  max(SCHAR_MIN, value / 4));   // convert to signed (zero = ADC/2)  
  position++;      
}

void ADCManager::stopCapture(){  
  //Serial.print("stopping capture ch");
  //Serial.println(channel);    
 
  ADCSRA &= ~_BV(ADEN);

  position = 0;
}

void ADCManager::postProcess(){
  ADCMax[channel] = -9999;
  ADCMin[channel] = 9999;  
  float avg = 0;
  for (int i=0; i < captureSize[channel]; i++){
    int8_t value = capture[channel][i];
    ADCMax[channel] = max(ADCMax[channel], value);
    ADCMin[channel] = min(ADCMin[channel], value);
    avg += ((float)value) / ((float)captureSize[channel]);
  }
  ADCAvg[channel] = avg;
}


void ADCManager::run(){
  if (busy) {
    //Serial.print("busy pos=");
    //Serial.println(position);
    return;
  }  
  if (position != 0){
    // stop free running
    stopCapture();    
    // post-process capture
    postProcess();
    captureComplete[channel]=true;          
    capturedChannels++;    
  }
  // find next channel for capturing
  for (int i=0; i < CHANNELS; i++){    
    channel++;
    if (channel == CHANNELS) channel = 0;
    if ((captureSize[channel] != 0) && (!captureComplete[channel])){        
      // found channel for sampling      
      startCapture( captureSize[channel] );                   
      break;
    }      
  }
}


int8_t* ADCManager::getCapture(byte pin){  
  return (int8_t*)capture[pin-A0];
}

boolean ADCManager::isCaptureComplete(byte pin){
  int ch = pin-A0;  
  return captureComplete[ch];        
}

int ADCManager::read(byte pin){    
  int ch = pin-A0;
  captureComplete[ch]=false;    
  if (captureSize[ch] == 0) return 0;  
    else return capture[ch][0];
}


void ADCManager::restart(byte pin){
  captureComplete[pin-A0]=false;
}
        
int ADCManager::getCapturedChannels(){
  int res = capturedChannels;
  capturedChannels=0;
  return res;
}

int ADCManager::getCaptureSize(byte pin){
  int ch = pin-A0;  
  return captureSize[ch];

}

int16_t ADCManager::getADCAvg(byte pin){
  int ch = pin-A0;  
  if (ch >= CHANNELS) return 0;
  return ADCAvg[ch];
}

int16_t ADCManager::getADCMin(byte pin){
  int ch = pin-A0;  
  if (ch >= CHANNELS) return 0;
  return ADCMin[ch];
}

int16_t ADCManager::getADCMax(byte pin){
  int ch = pin-A0;  
  if (ch >= CHANNELS) return 0;
  return ADCMax[ch];
}

int16_t ADCManager::getADCOfs(byte pin){
  int ch = pin-A0;  
  if (ch >= CHANNELS) return 0;
  return ofs[ch];
}  

void ADCManager::loadSaveCalib(boolean readflag){
  int addr = ADDR;
  short magic = MAGIC;
  eereadwrite(readflag, addr, magic); // magic
  for (int ch=0; ch < CHANNELS; ch++){
    eereadwrite(readflag, addr, ofs[ch]);
  }  
}

boolean ADCManager::loadCalib(){
  short magic = 0;
  int addr = ADDR;
  eeread(addr, magic);
  if (magic != MAGIC) {
    Serial.println(F("ADCMan error: no calib data"));
    return false;   
  }
  calibrationAvail = true;
  Serial.println(F("ADCMan: found calib data"));
  loadSaveCalib(true);
  return true;
}

void ADCManager::saveCalib(){
#ifdef __AVR__
  loadSaveCalib(false);
#endif
}
