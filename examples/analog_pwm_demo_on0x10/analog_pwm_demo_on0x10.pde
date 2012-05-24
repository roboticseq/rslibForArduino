/****
Reads an analog input on pin 4 of a on a Robotic Sequencing I2C board on address 0x10
Use a switch on pin 2 configured as a pulldown, to change R-G-B value from analog input on pin 4.
then prints ajust the RGB value of a RGB led using pwm. 
Red is pin 0, Green is pin 1, Blue is pin 5, Switch on pin 2.

by Stephane Rousseau <http://www.roboticsequencing.com>

Demonstrates use of the RS library
Read an analog value via I2C, update pwm.
Created 2012 February 28th
 
Copyright (C) 2012 Stephane Rousseau

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), 
to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, 
and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
****/


#include <Wire.h>
#include <RS.h>

uint8_t boardAddr = 0x10; // rsboard with a i2c address 0x10 (default address)
uint8_t analogPin = 4; // pin 4
uint8_t switchPin = 2; // pin 2
uint8_t switchPos = 0; // 0 = adjust RED, 1 = adjust green, 2 = adjust blue
uint8_t RedOutPin = 0; // pin 0 - shared pwm (cannot be used while a servo is used on any other pin on a rsboard)
uint8_t GreenOutPin = 1; // pin 1 - dedicated pwm
uint8_t BlueOutPin = 5; // pin 5 - dedicated pwm

RSDigitalPin dPin = RSDigitalPin(); // pin can be reused because no state is conserved.
RSAnalogPin aPin = RSAnalogPin();
RSPwmPin pwmPin = RSPwmPin();

int debounceSwitchVal = HIGH;
int debounceTime = 50; // 50 milliseconds
int lastSwitchVal;
int lastDebounceTime = 0;



void setup() {
  //Serial.begin(38400);
  dPin.setMode(boardAddr,switchPin, INPUT); // set pin a input
  dPin.write(boardAddr, switchPin, HIGH); // mode pullup
  dPin.setMode(boardAddr,RedOutPin, OUTPUT);
  dPin.setMode(boardAddr,GreenOutPin, OUTPUT);
  dPin.setMode(boardAddr,BlueOutPin, OUTPUT);
}

void loop() {

  // debouce switch reading
  int switchVal = 0;
  
  switchVal = dPin.read(boardAddr,switchPin);
  
  if (switchVal != lastSwitchVal) {
    // reset the debounce timer
    lastDebounceTime = millis();
  } 
  
  if ((millis() - lastDebounceTime) > debounceTime) {
    // val has stabilized
    debounceSwitchVal = switchVal;
  }
  
  lastSwitchVal = switchVal;
  
  // end debouce switch read
  
  if ( debounceSwitchVal == 0 )
  {
    // switch press detected
    if ( switchPos == 2 )
      {
		switchPos = 0;
      }
    else
      {
		switchPos++;
      }
	//Serial.print("Switch pos:");
	//Serial.println(switchPos,DEC);
  }
 
 int sensorValue = (aPin.read(boardAddr,analogPin)); // read and put the result on 8 bits
 switch (switchPos) {
  case 0:
    pwmPin.write(boardAddr, RedOutPin, sensorValue/4);
    break;
  case 1:
    pwmPin.write(boardAddr, GreenOutPin, sensorValue/4);
    break;
  case 2: 
    pwmPin.write(boardAddr, BlueOutPin, sensorValue/4);
	break;
  }
 
 
  //Serial.print("ADC Pin 4:");
  //Serial.println(sensorValue, DEC);

  // wait for 100 milliseconds to see the dimming effect    
  delay(100);   
}

