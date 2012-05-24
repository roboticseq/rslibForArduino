/****
 Step Motor example using a EasyDriver
This example can be adapted to step motor driver board who use DIR STEP approach.
Like EasyDriver, Pololu A4988 Stepper Motor Driver,...

by Stephane Rousseau <http://www.roboticsequencing.com>

Demonstrate use of the RS library
To move a step motor 1600 steps

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



#define ENABLEPIN 7
#define DIRPIN 6 
#define STEPPIN 5
#define REMOTEADDR 0x10
RSDigitalPin ePin = RSDigitalPin();
RSStepper rsStep = RSStepper();

void setup() {
 Serial.begin(38400);
 Serial.println("RSI2C Step Motor Demo using dirStep pulse like Easydriver");
 Serial.println("dirstep pinout: step = P5, dir = P6, ena = P7");
 Serial.println("Valid command are:");
 Serial.println("f= 1600 steps forward, r=1600 steps backward, s=stop with stepper motor in floating mode");
 Serial.println("l= 32000 steps forward to be able to read the number of steps to go.");
 ePin.setMode(REMOTEADDR, ENABLEPIN, OUTPUT);
 ePin.write(REMOTEADDR, ENABLEPIN, LOW); // on easydriver enable logic is low = on, high = off
 rsStep.setPins(REMOTEADDR,DIRPIN, STEPPIN);
 rsStep.setProfile(REMOTEADDR,600,6400,1000,1000);//optional default is 800,1600,6400,6400 if not used or changed in startup script on board.
}

void loop() {
	uint8_t incomingByte;
	int32_t toGo;
    if (Serial.available() > 0) {
		// read the incoming byte:
        incomingByte = Serial.read();
        if ( incomingByte == 'f') {
			ePin.write(REMOTEADDR, ENABLEPIN, LOW);
			rsStep.move(REMOTEADDR, 1600); // 4 turn forward
        }
        else if ( incomingByte == 'r') {
			ePin.write(REMOTEADDR, ENABLEPIN, LOW);
            rsStep.move(REMOTEADDR,-1600); // 4 turn reverse
        }
		else if ( incomingByte == 's') {
			ePin.write(REMOTEADDR, ENABLEPIN, HIGH);
		}
		else if ( incomingByte == 'l') {
			ePin.write(REMOTEADDR, ENABLEPIN, LOW);
			rsStep.move(REMOTEADDR,32000);
		}
		else if ( incomingByte == 't') {
			// print remaining distance
			toGo = rsStep.getToGo(REMOTEADDR);
			Serial.print("Steps to go=");
			Serial.println(toGo,DEC);
		}
	}
}