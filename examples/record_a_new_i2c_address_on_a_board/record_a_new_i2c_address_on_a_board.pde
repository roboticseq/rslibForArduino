/*****
Record_a_new_i2c_address_on_a_board utility program

by Stephane Rousseau <http://www.roboticsequencing.com>

Demonstrate use of the RS library
to record a script who changes the i2c address of the board at startup.

Created 2012 February 1st

Copyright (C) 2012 Stephane Rousseau

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), 
to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, 
and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*****/

#include <Wire.h>
#include <RS.h>
#include <RSutil.h>

#define DEFAULTADDR 0x10
RSRecorder rsRec = RSRecorder();
RSDictionnary rsDict = RSDictionnary();

#define readBufMaxLen 20
uint8_t readBuffer[readBufMaxLen];
uint8_t readLen = 0;
uint8_t resp;
int fromI2CAddress = 0;
int toI2CAddress = 0;
boolean done = false;

void setup() {
 Serial.begin(38400);
 Serial.println("RSI2C I2C Address Change Utility Program");
 Serial.println("If you don't know the current address, isolate the board on the i2c bus, ");
 Serial.println("And use the broadcast address 00");
 Serial.println("Enter current address of the board (16 is default board, 00 is broadcast:");
 
 readLen = RSreadLine( &readBuffer[0], readBufMaxLen, true);

 fromI2CAddress = RSstrToInt( &readBuffer[0] , readLen);
 
 Serial.println("Enter new address of the board (16 is default board, 00 is broadcast:");
 
 readLen = RSreadLine( &readBuffer[0], readBufMaxLen, true);
 toI2CAddress = RSstrToInt( &readBuffer[0] , readLen);
 
 Serial.println("");
 Serial.print("Changing RSI2Cboard address from: ");
 Serial.print(fromI2CAddress, DEC);
 Serial.print("(0x");
 Serial.print(charHexToString( fromI2CAddress ));
 Serial.print(") to address: ");
 Serial.print(toI2CAddress, DEC);
 Serial.print("(0x");
 Serial.print(charHexToString( toI2CAddress ));
 Serial.println(") ");
 Serial.println("Do you want to proceed y/n");
 while (!done)
 {
	if (Serial.available()){
		resp = Serial.read();
		if (resp == 'y')
		{
			Serial.println("Proceeding RSI2Cboard address update!");
			done = true;			
			rsRec.scriptRecordStart(DEFAULTADDR); // the scriptRecordStart and Stop can be use to record more advanced script on a board
			rsDict.changeI2CBoardAddress(DEFAULTADDR,toI2CAddress);
			rsRec.scriptRecordStop(DEFAULTADDR);
			Serial.println("DONE: Address changed power down and up the board to use the new address.");
		}
		else	if ( resp == 'n'){
			Serial.println("Abording RSI2Cboard address update!");
			done = true;
		}
	}
 };
 
}

void loop() {

}

