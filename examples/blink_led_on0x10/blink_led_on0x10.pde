/****
Blink led on a Robotic Sequencing I2C board on address 0x10

Demonstrates use of the RS library
Blink a led via I2C on a RSi2cBoard at 0x10 address.

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

#define LEDP1 1
#define REMOTEADDR 0x10


RSDigitalPin ledPin = RSDigitalPin();


void setup()
{
	ledPin.setMode(REMOTEADDR, LEDP1, OUTPUT);
}


void loop() // run over and over again
{
ledPin.write(REMOTEADDR, LEDP1, HIGH); // sets the LED on
delay(1000); // waits for a second
ledPin.write(REMOTEADDR, LEDP1, LOW); // sets the LED off
delay(500); // waits for a half second
}

