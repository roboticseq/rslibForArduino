/****
Reads an regular encoder on pin 1 on a Robotic Sequencing I2C board on address 0x10
then prints the result to the serial monitor.
 
by Stephane Rousseau <http://www.roboticsequencing.com>

Demonstrates use of the RS library

Created 2012 January 29th

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

// Counter on RSI2Cboard PinA must be pin 1 
#define PINA 1
#define REMOTEADDR 0x10


RSCounter rsCount = RSCounter();
RSDigitalPin rsPinA = RSDigitalPin(); // to put pinA in pullup mode


void setup() {
  Serial.begin(38400);
  rsCount.setPins(REMOTEADDR, PINA, 0xff);
  rsPinA.write(REMOTEADDR, PINA, HIGH); // sets the LED on
  rsCount.resetCount(REMOTEADDR); // reset count to zero;
}

void loop() {
  int32_t countValue = rsCount.getCount(REMOTEADDR);
  Serial.println(countValue);
  delay(1000); // wait 1 sec
}
