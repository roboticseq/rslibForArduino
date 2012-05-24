/****
Demonstrates use of the RS library
Write an analog value using pwm8 on pin 0 
 
by Stephane Rousseau <http://www.roboticsequencing.com>

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
uint8_t analogPin = 0; // pin 0
int brightness = 0;    // how bright the LED is
int fadeAmount = 5;    // how many points to fade the LED by

RSPwmPin pwmPin = RSPwmPin();

void setup() 
{

}

void loop() 
{

	// set the brightness of analog pin out:  
	pwmPin.write(boardAddr, analogPin, brightness);  

	// change the brightness for next time through the loop:
	brightness = brightness + fadeAmount;

	// reverse the direction of the fading at the ends of the fade: 
	if (brightness == 0 || brightness == 255) {
		fadeAmount = -fadeAmount ; 
	}     
	// wait for 100 milliseconds to see the dimming effect    
	delay(100);   
}



