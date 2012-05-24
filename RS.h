/*!
  \fileRS.h - Robotic Sequencing library for Arduino & Wiring
  Copyright (c) 2011-2012 Stephane Rousseau.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef RS_h
#define RS_h

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
  #else
  #include "WProgram.h"
  #endif
  
#include <Wire.h>
#include <RSutil.h>

#define RSMSGLEN 4
#define RSRESPLEN 3


// pin change mode 
#define RSPINLOW 0
#define RSPINANY 1
#define RSPINRISINGEDGE 2
#define RSPINFALLINGEDGE 3


enum RSBoardError { RSBOARD_SUCCESS = 0,
				 RSBOARD_UNKNOWN_ERROR, 
				 RSBOARD_LAST_MOVE_NOTCOMPLETED_ERROR,
				 RSBOARD_SP0MUSTBEEQUALORLOWERTHANSPMAX_ERROR,
				 RSBOARD_ACCELANDDECELMUSTBEPOSITIVEGREATERTHANZERO,
				 RSBOARD_LAST_ERROR
						};

/// Some utility functions
void RSLoadAcc( uint8_t addr, uint16_t val);



class RSComChannel
{
private:
	boolean _initDone;
public:
	RSComChannel();
	void init();
	void writeMessage(uint8_t boardAddress, uint8_t * msg, const uint8_t len);
	uint8_t readResponse(uint8_t boardAddress, uint8_t * resp, const uint8_t len);
};

/****
	All pin based classes use no state information, this allow reused of pin objects, permit
	a fonctionnal based type programming style and reduce the number of objects on the Arduino.
	This approach is used since there is no dynamic allocation on the Arduino.
*****/
class RSDigitalPin
{
public:
	RSDigitalPin();
	static void setMode( uint8_t addr, uint8_t pin, uint8_t mode);
	static void write( uint8_t addr, uint8_t pin_t, uint8_t value);
	static uint8_t read( uint8_t addr, uint8_t pin_t);
};

class RSPwmPin
{
public:
	RSPwmPin();
	static void write( uint8_t addr, uint8_t pin, uint8_t value);
};

class RSAnalogPin 
{
public:
	RSAnalogPin();
	static int16_t read( uint8_t addr, uint8_t pin);
};

class RSServo
{
public:
	RSServo();
	static void writeUs(uint8_t addr, uint8_t pin, uint16_t posUs);
	static void write(uint8_t addr, uint8_t pin, uint16_t posDegreeOrUs);
	static void detach(uint8_t addr, uint8_t pin);
};

/*****
	Each board can contain a dictionnary to acces value-pair data
*****/
class RSDictionnary
{
public:
	RSDictionnary();
	static boolean read(uint8_t addr, int16_t key, int16_t *readVal); // retrieve a value from the dictionnary using a key
	static uint8_t write(uint8_t addr, int16_t key, int16_t writeVal);// write a key-value pair into the dictionnary 
	static uint8_t deleteKeyVal(uint8_t addr, int16_t key); // delete a key-value pair
	static uint8_t deleteAll(uint8_t addr); // delete all the dictionnary entries
	static void changeI2CBoardAddress(uint8_t addr, uint8_t newAddress); // change the address of the board
};

/****
   RSRecorder enable the use of a Arduino sketch to record startup script in a RSI2CBoard to enable autonomous
   program in a board.
****/
class RSRecorder
{
public:
	RSRecorder();
	static void scriptRecordStart(uint8_t addr); // the scriptRecordStart and Stop can be use to record more advanced script on a board
	static void scriptRecordStop(uint8_t addr);
};

class RSCounter
{
public:
	RSCounter();
	static void setPins( uint8_t addr, uint8_t pinA, uint8_t pinB);
	static void setPinATriggerType( uint8_t addr, uint8_t mode);
	static void resetCount( uint8_t addr );
	static int32_t getCount( uint8_t addr);
};

class RSStepper
{
public:
	RSStepper();
	static void setPins( uint8_t addr, uint8_t dirPin, uint8_t stepPin);
	static uint8_t setProfile( uint8_t addr, int16_t sp0, int16_t spMax, int16_t accel, int16_t decel);
	static void move( uint8_t addr, int32_t steps);
	static int32_t getToGo( uint8_t addr);
};



// rsCom is a global object already instanciated

extern RSComChannel rsCom;

#endif

