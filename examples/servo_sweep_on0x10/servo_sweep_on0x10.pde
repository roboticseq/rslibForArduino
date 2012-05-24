/****
Sweep a servo on a Robotic Sequencing I2C board on address 0x10
by Stephane Rousseau <http://www.roboticsequencing.com>

Demonstrates use of the RS library
Controls a servo via I2C

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
#define REMOTEADDR 0x10

RSServo servo = RSServo();
 
int pos = 0;    // servo position

void setup() 
{ 
  //Serial.begin(38400);
} 
 
 
void loop() 
{ 
  
  //Serial.println("Sweep phase 0 to 180");

  for(pos = 0; pos < 180; pos += 1)  // goes from 0 degrees to 180 degrees 
  {     // in steps of 1 degree 
    servo.write(REMOTEADDR, SERVOPIN, pos); // update the position of the servo
    delay(15);                       // waits 15ms for the servo to reach the position 
  } 
  
  //Serial.println("Sweep phase 180 to 0");
  for(pos = 180; pos>=1; pos-=1)     // goes from 180 degrees to 0 degrees 
  {    
    servo.write(REMOTEADDR, SERVOPIN, pos); // update the position of the servo 
    delay(15);                       // waits 15ms for the servo to reach the position 
  }

} 

