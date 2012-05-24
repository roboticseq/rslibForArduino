/****
Reads an analog input on pin 4 of a on a Robotic Sequencing I2C board on address 0x10
then prints the result to the serial monitor.

Demonstrates use of the RS library
Read an analog value via I2C

by Stephane Rousseau <http://www.roboticsequencing.com>
Created 2012 January 28th

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
uint8_t analogPin = 4; // analog will be read on pin 4
RSAnalogPin aPin = RSAnalogPin();

void setup() {
  Serial.begin(38400); // open serial port
  
}

void loop() {
  int sensorValue = (aPin.read(boardAddr,analogPin)); // read 
  
  Serial.println(sensorValue, DEC); // print the value
  delay(200); // wait 200ms
}



