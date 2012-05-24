/****
Reads an analog input on pin 2 of a on a Robotic Sequencing I2C board on address 0x10
Update position a of servo on pin 6 from value read on pin 2.

 
by Stephane Rousseau <http://www.roboticsequencing.com>

Demonstrate use of the RS library
Read an analog value via I2C, update a servo position

Created 2012 February 29th

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

#define SERVOPIN 7
#define ANALOGPIN 4
#define REMOTEADDR 0x10


RSAnalogPin analogPin = RSAnalogPin();
RSServo servo = RSServo();

void setup() {
 //Serial.begin(38400);
}

void loop() {

 int sensorValue = analogPin.read(REMOTEADDR, ANALOGPIN);; // read and put the result on 8 bits
 //Serial.print("ADC Pin 4:");
 //Serial.println(sensorValue, DEC);
 sensorValue = map(sensorValue, 0, 1023, 0, 179);
 servo.write(REMOTEADDR, SERVOPIN, sensorValue); // update the position of the servo

 
 //Serial.print("Mapped value for the servo:");
 //Serial.println(sensorValue, DEC);
 // wait for 100 milliseconds to see the dimming effect    
 delay(20);   
}