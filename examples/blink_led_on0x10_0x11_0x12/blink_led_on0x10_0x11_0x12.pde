/****
Blink led on a Robotic Sequencing I2C board on address 0x10
by Stephane Rousseau <http://www.roboticsequencing.com>

Demonstrates use of the RS library
Controls leds via I2C on multiple board

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

#define LED0 0
#define LED1 1
#define LED2 2
#define BOARD16 0x10
#define BOARD17 0x11
#define BOARD18 0x12

RSDigitalPin ledPin = RSDigitalPin();

void setup()
{
// show the reuse of the stateless RSDigitalPin
ledPin.setMode(BOARD16, LED0, OUTPUT);
ledPin.setMode(BOARD17, LED1, OUTPUT);
ledPin.setMode(BOARD18, LED2, OUTPUT);
}


void loop() // run over and over again
{
ledPin.write(BOARD16, LED0, HIGH); // sets the LED on
ledPin.write(BOARD17, LED1, HIGH);
ledPin.write(BOARD18, LED2, HIGH);
delay(1000); // waits for a second
ledPin.write(BOARD16, LED0, LOW); // sets the LED off
ledPin.write(BOARD17, LED1, LOW);
ledPin.write(BOARD18, LED2, LOW);
delay(500); // waits for a half second
}

