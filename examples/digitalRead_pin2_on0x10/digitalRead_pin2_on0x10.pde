/****
Read a switch on pin 2 using internal pullup and turn on/off a led on a Robotic Sequencing I2C board on address 0x10

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

#define LED 0
#define SWITCH 2
uint8_t boardAddr = 0x10; // rsboard with a i2c address 0x10 (default address)




RSDigitalPin ledPin = RSDigitalPin();
RSDigitalPin switchPin = RSDigitalPin();


void setup()
{
	ledPin.setMode(boardAddr, LED, OUTPUT);
	switchPin.setMode(boardAddr, SWITCH, INPUT);
	switchPin.write(boardAddr, SWITCH, HIGH); // set input pin high to enable internal pullup
}


void loop() // run over and over again
{

uint16_t readVal = 0;

readVal = switchPin.read(boardAddr,SWITCH);

// pullup mode 1=switch open 0= switch close
if ( readVal) 
{
	ledPin.write(boardAddr, LED, LOW); // sets the LED on
}
else
{
	// switch pressed detected
	ledPin.write(boardAddr, LED, HIGH); // sets the LED off
}
delay(100); // waits for a 1/10th second
}

