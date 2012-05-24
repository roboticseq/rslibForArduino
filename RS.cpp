/*!
  \file RS.cpp - Robotic Sequencing library for Wiring & Arduino
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

#include "RS.h"

// Initialize Class Variables //////////////////////////////////////////////////
#define RSDEFAULT_ADDRESS 0x10
#define RSDEFAULT_TYPE RSI2C
#define RSREADDELAY 1

//#define DEBUG

/// rsutility function
void RSLoadAcc( uint8_t addr, uint16_t val)
{
	uint8_t msg[RSMSGLEN];
	msg[0] = 'l';
	msg[1] = 0x00;
	msg[2] = val >> 8;
	msg[3] = val & 0xff;
	rsCom.writeMessage(addr, msg,RSMSGLEN); // send the complete message
}



#if defined(ARDUINO)
// Arduino RSComChannel implementation

RSComChannel::RSComChannel()
{
	_initDone = false;
}

void RSComChannel::init( void )
{
	delay(100); // waits for 1/10 second
	Wire.begin(); // join i2c bus (address optional for master)
	_initDone = true;
}

void RSComChannel::writeMessage(uint8_t boardAddress, uint8_t * msg, const uint8_t len)
{
	#ifdef DEBUG
		Serial.print("Write message to address: ");
		Serial.print(boardAddress,DEC);
		Serial.print(" message: 0x");
	#endif

	if (! _initDone )
	{
		init();
	}

	Wire.beginTransmission(boardAddress); // transmit to rsi2c board address
	for( uint8_t i = 0; i < len; i++)
	{
		#if defined(ARDUINO) && ARDUINO >= 100
			Wire.write(msg[i]); 
		#else
			Wire.send(msg[i]); 
		#endif
		
		#ifdef DEBUG
			Serial.print(charHexToString(msg[i]));
		#endif
	}
	Wire.endTransmission(); 
	#ifdef DEBUG
		Serial.println("");
	#endif
}

uint8_t RSComChannel::readResponse(uint8_t boardAddress, uint8_t * resp, const uint8_t len)
{
		uint8_t cnt = 0;

	#ifdef DEBUG
		Serial.print("Read n byte address: ");
		Serial.print(boardAddress,DEC);
		Serial.print(" read: 0x");
	#endif

	if (! _initDone )
	{
		init();
	}

	delay(RSREADDELAY); // waits for 1/1000 second to allow bus to stabilize before read
	
	Wire.requestFrom((uint8_t) boardAddress, (uint8_t) len);    // request 4 bytes from slave device
	
	while((Wire.available() > 0) && (cnt < len))    // slave may send less than requested but we should not accept more
	{
		#if defined(ARDUINO) && ARDUINO >= 100
			resp[cnt] = Wire.read(); // receive a byte
		#else
			resp[cnt] = Wire.receive(); // receive a byte
		#endif
		#ifdef DEBUG
			Serial.print(charHexToString(resp[cnt]));
		#endif
		cnt++;
	}
	
	#ifdef DEBUG
	Serial.println("");
	#endif
	return cnt;
}
#endif


// Class RSDigitalIOPin implementation
/*!
 * \class RSDigitalPin
 *
 * \brief RSDigitalPin class to acces digital pin on a RSi2cBoard.
 *
 * RSDigitalPin is static a class and doesn't use any state variable, 
 * only one instance is required.
 *
 * \author Stephane Rousseau
 * \date
 */
RSDigitalPin::RSDigitalPin()
{
}

/*!
 * \brief Set the mode the digital pin
 *
 * Set pin as INPUT or OUTPUT.
 *
 * \param[in] addr address of the board, 0x10 is the default board I2C address
 * \param[in] pin pin on the board
 * \param[in] mode INPUT or OUTPUT
 * \sa read
 */
void RSDigitalPin::setMode( uint8_t addr, uint8_t pin, uint8_t mode)
{
	uint8_t msg[RSMSGLEN];
  
	if (mode == INPUT)
	{
		msg[0] = 'i';             // input
	}
	else if (mode == OUTPUT)
	{
		msg[0] = 'o';            // output
	}
   
	msg[1] = 'm';
	msg[2] = pin;
	msg[3] = 0x00;
	rsCom.writeMessage(addr, msg,RSMSGLEN); // send the complete message
}

/*!
 * \brief Write a digital pin
 *
 * Write a digital pin, can be used to output a level HIGH, LOW on a pin or can be 
 * used to put an input pin in pullup or pulldown state.
 *
 * \param[in] addr address of the board, 0x10 is the default board I2C address
 * \param[in] pin pin on the board
 * \param[in] value HIGH or LOW
 * \sa read
 */
void RSDigitalPin::write( uint8_t addr, uint8_t pin, uint8_t value)
{
	uint8_t msg[RSMSGLEN];
	msg[0] = 'o';
	msg[1] = 'w';
	msg[2] = pin;
	if ( value == 0 ){
		msg[3] = 0;
	} else {
		msg[3] = 1;
	}
	rsCom.writeMessage(addr,msg,RSMSGLEN); // send the complete message
}

/*!
 * \brief Read a digital pin
 *
 * Read a digital pin.
 *
 * \param[in] addr address of the board, 0x10 is the default board I2C address
 * \param[in] pin pin on the board
 * \return HIGH or LOW
 * \sa read
 */
uint8_t RSDigitalPin::read( uint8_t addr, uint8_t pin)
{
	int cnt = 0;
	int res = 0;
	uint8_t resp[RSRESPLEN];
	uint8_t msg[RSMSGLEN];
	
	msg[0] = 'i';
	msg[1] = 'r';
	msg[2] = pin;
	msg[3] = 0x00;
	
	rsCom.writeMessage(addr,msg,RSMSGLEN); // send the complete message
	
	resp[0]=0x00;

	cnt= rsCom.readResponse(addr,&resp[0], RSRESPLEN);

	//interpret the result
	if (cnt == 3 && resp[0] == 'i')
	{
		res = (resp[1] << 8) + resp[2]; // big endian
	}
	else
	{
		// error return 0
		res = 0;
	}
  
	return res;
}

// Class RSPwmPin implementation
/*!
 * \class RSPwmPin
 *
 * \brief RSPwmPin class to acces pwm pin on a RSi2cBoard.
 *
 * RSPwmPin is static a class and doesn't use any state variable, 
 * only one instance is required.
 *
 * \author Stephane Rousseau
 * \date
 */
RSPwmPin::RSPwmPin()
{
}

/*!
 * \brief Write a pwm pin
 *
 * Write a pwm pin, can be used to output a level from 0 to 255 on a pwm pin.
 * On a RSi2cBoard, up to 3 pins can be used simultaneously, pin 1, pin 5 and
 * one other pin on a board. Pin 1 and 5 used a dedicated 8 bits counter.
 *
 * \param[in] addr address of the board, 0x10 is the default board I2C address
 * \param[in] pin pin on the board
 * \param[in] value 0..255
 */

void RSPwmPin::write( uint8_t addr, uint8_t pin, uint8_t value)
{
	uint8_t msg[RSMSGLEN];
	msg[0] = 'p';
	msg[1] = 'w';
	msg[2] = pin;
	msg[3] = value & 0xff;
	rsCom.writeMessage(addr,msg,RSMSGLEN); // send the complete message
}

// Class RSAnalogPin implementation
/*!
 * \class RSAnalogPin
 *
 * \brief RSAnalogPin class to read an analog value on a pin on a RSi2cBoard.
 *
 * RSAnalogPin is static a class and doesn't use any state variable, 
 * only one instance is required.
 *
 * \author Stephane Rousseau
 * \date
 */
RSAnalogPin::RSAnalogPin()
{
}

/*!
 * \brief Read a analog pin
 *
 *  Pin 0 to Pin 7 can be read.
 *
 * \param[in] addr address of the board, 0x10 is the default board I2C address
 * \param[in] pin pin on the board
 * \return a 10 bits value from 0..1023
 */
int16_t RSAnalogPin::read( uint8_t addr, uint8_t pin)
{
	int cnt = 0;
	int res = 0;
	uint8_t resp[RSRESPLEN];
	uint8_t msg[RSMSGLEN];
	
	msg[0] = 'a';
	msg[1] = 'r';
	msg[2] = pin;
	msg[3] = 0x00;
	
	rsCom.writeMessage(addr,msg,RSMSGLEN); // send the complete message
	
	resp[0]=0x00;

	cnt= rsCom.readResponse(addr,&resp[0], RSRESPLEN);

  //interpret the result
  if (cnt == 3 && resp[0] == 'a')
  {
	res = (resp[1] << 8) + resp[2]; // big endian
  }
  else
  {
     // error return 0
	res = 0;
  }
  
  return res;
}

// Class RSServo implementation
// standard pulse duration from Hitec spec 
#define RSSERVOMINUS 900
#define RSSERVOMAXUS 2100

/*!
 * \class RSServo
 *
 * \brief RSServo class use a pin as a RC Servo on a RSi2cBoard.
 *
 * Can use up to 8 servos on a RSi2cBoard.
 *
 * \author Stephane Rousseau
 * \date
 */
RSServo::RSServo()
{
}

/*!
 * \brief Write servo position using a uS value 
 *
 * Typical value are from 900..2100 us
 *
 * \param[in] addr address of the board, 0x10 is the default board I2C address
 * \param[in] pin pin on the board
 * \param[in] value 0..3000
 * \sa writeUs
 */
void RSServo::writeUs(uint8_t addr, uint8_t pin, uint16_t posUs)
{
  	uint8_t msg[RSMSGLEN];
	msg[0] = 'r';
	msg[1] = pin;
	msg[2] = (uint8_t)((posUs & 0xff00) >> 8);
	msg[3] = (uint8_t)(posUs & 0xff);
	rsCom.writeMessage(addr,msg,RSMSGLEN); // send the complete message
}

/*!
 * \brief Write servo position using a degree or Us value 
 *
 * If value is less than 181 it's assumed as a degrees position, if not it's a uS position.
 *
 * \param[in] addr address of the board, 0x10 is the default board I2C address
 * \param[in] pin pin on the board
 * \param[in] value 0..3000
 * \sa writeUs
 */
void RSServo::write(uint8_t addr, uint8_t pin, uint16_t posDegreeOrUs)
{
	uint16_t posUs = 0;

	if ( posDegreeOrUs < 181)
	{
		posUs = map(posDegreeOrUs, 0, 180, RSSERVOMINUS,  RSSERVOMAXUS);
		// use the writUs method.
		writeUs( addr, pin, posUs);
	}
	else if ( posDegreeOrUs > RSSERVOMINUS && posDegreeOrUs < RSSERVOMAXUS)
	{
		posUs = posDegreeOrUs;
		// use the writUs method.
		writeUs( addr, pin, posUs);
	}
	
	
}

/*!
 * \brief Turn off the servo 
 *
 *
 * \param[in] addr address of the board, 0x10 is the default board I2C address
 * \param[in] pin pin on the board
 */
void RSServo::detach(uint8_t addr, uint8_t pin)
{
	// use the writUs method with 0uS who turn it off 
	writeUs( addr, pin, 0);
}

/*!
 * \class RSDictionnary
 *
 * \brief A small key-value store on the board.
 *
 * Up to 40 entries can be stored in a RSi2cBoard v1.
 *
 * \author Stephane Rousseau
 * \date
 */
RSDictionnary::RSDictionnary()
{
}

/*!
 * \brief Retrieve a value from the dictionnary using a key
 *
 *  Return true, if the key-value pair is found.
 *
 * \param[in] addr address of the board, 0x10 is the default board I2C address
 * \param[in] key key of the value to be retrieved
 * \param[out] *value a pointer to the value variable.
 * \return TRUE if succes, FALSE if the value is not found.
 */
boolean RSDictionnary::read(uint8_t addr, int16_t key, int16_t *readVal)
{
	int cnt = 0;
	boolean res = 0;
	int16_t val = 0;
	uint8_t resp[RSRESPLEN];
	uint8_t msg[RSMSGLEN];

	msg[0] = 'd';
	msg[1] = 'r';
	msg[2] = key >> 8;
	msg[3] = key & 0xff;
	rsCom.writeMessage(addr,msg,RSMSGLEN); // send the complete message
	
	cnt= rsCom.readResponse(addr,&resp[0], RSRESPLEN);

  //interpret the result
  if (cnt == 3 && resp[0] == 'd')
  {
	val = resp[1] << 8 | resp[2] & 0xff; 
	res = true; // big endian
  }
  else
  {
     // error return 0
	res = false;
  }
	
	return res;
}

  
/*!
 * \brief Write a key-value pair into the dictionnary
 *
 *
 * \param[in] addr address of the board, 0x10 is the default board I2C address
 * \param[in] key key of the value to be stored or updated.
 * \param[out] value value to be stored

 */
uint8_t RSDictionnary::write(uint8_t addr, int16_t key, int16_t writeVal)
{
	uint8_t msg[RSMSGLEN];
	
	RSLoadAcc( addr, writeVal);
	
	msg[0] = 'd';
	msg[1] = 'w';
	msg[2] = key >> 8;
	msg[3] = key & 0xff;
	rsCom.writeMessage(addr,msg,RSMSGLEN); // send the complete message
}


/*!
 * \brief Delete a key-value pair
 *
 *
 * \param[in] addr address of the board, 0x10 is the default board I2C address
 * \param[in] key key of the value to be deleted
 */
uint8_t RSDictionnary::deleteKeyVal(uint8_t addr, int16_t key)
{
	uint8_t msg[RSMSGLEN];
	msg[0] = 'd';
	msg[1] = 'd';
	msg[2] = key >> 8;
	msg[3] = key & 0xff;

	rsCom.writeMessage(addr,msg,RSMSGLEN); // send the complete message
}

/*!
 * \brief delete all the dictionnary entries
 *
 *
 * \param[in] addr address of the board, 0x10 is the default board I2C address
 */
uint8_t RSDictionnary::deleteAll(uint8_t addr)
{
	uint8_t msg[RSMSGLEN];
	
	msg[0] = 'd';
	msg[1] = 'e';
	msg[2] = 0x00;
	msg[3] = 0x00;
	
	rsCom.writeMessage(addr,msg,RSMSGLEN); // send the complete message
}


/*!
 * \brief Change the address of the board
 *
 *
 * \param[in] addr address of the board, 0x10 is the default board I2C address
 * \param[in] newAddress 1..127
 */
void RSDictionnary::changeI2CBoardAddress(uint8_t addr, uint8_t newAddress)
{
	write( addr, ('$' << 8 | 'i'), newAddress );
}

/*!
 * \class RSRecorder
 *
 * \brief RSRecorder allow a script to be recorded on a board.
 *
 * RSRecorder can be used to record more advanced script on a boardUp  in EEPROM. 
 * Up to 100 instruction can be recorded on a RSi2cBoard v1.
 *
 * \author Stephane Rousseau
 * \date
 */
RSRecorder::RSRecorder()
{
}


/*!
 * \brief Start recording
 *
 *
 * \param[in] addr address of the board, 0x10 is the default board I2C address
 */
void RSRecorder::scriptRecordStart(uint8_t addr)
{
	uint8_t msg[RSMSGLEN];
	
	msg[0] = 's';
	msg[1] = 'r';
	msg[2] = 's';
	msg[3] = 0x00;
	
	rsCom.writeMessage(addr,msg,RSMSGLEN); // send the complete message
}

/*!
 * \brief Stop recording
 *
 *
 * \param[in] addr address of the board, 0x10 is the default board I2C address
 */
void RSRecorder::scriptRecordStop(uint8_t addr)
{
	uint8_t msg[RSMSGLEN];
	
	msg[0] = 's';
	msg[1] = 'r';
	msg[2] = 'e';
	msg[3] = 0x00;
	
	rsCom.writeMessage(addr,msg,RSMSGLEN); // send the complete message
}


/*!
 * \class RSCounter
 *
 * \brief RSCounter
 *
 * RSCount can be used to count event on a pin. Counter can be a regular counter or a quadrature counter. 
 *
 * \author Stephane Rousseau
 * \date
 */
RSCounter::RSCounter()
{
}

/*!
 * \brief Set the pins of the counter.
 *
 *
 * \param[in] addr address of the board, 0x10 is the default board I2C address
 * \param[in] pinA must be 1 on a RSi2cBoard v1.
 * \param[in] pinB second pin on a quadrature counter. 0xff indicate a regular counter.
 */
void RSCounter::setPins( uint8_t addr, uint8_t pinA, uint8_t pinB)
// assing pin a and pinb to the a counter
// if pinb is 0xff this will be a regular counter, else it's a quadrature counter
{

	uint8_t msg[RSMSGLEN];
	

	msg[0] ='c';            // sends instruction counter 			
	msg[1] ='p';             // sub: pins configuration
	msg[2] = pinA;   
	msg[3] = pinB;

	rsCom.writeMessage(addr,msg,RSMSGLEN); // send the complete message

}

/*!
 * \brief Set pinA trigger type. Which event trigger the count.
 *
 *
 * \param[in] addr address of the board, 0x10 is the default board I2C address
 * \param[in] mode triger mode LOW = 0, ANY = 1, FAILINGEDGE = 2, RAISINGEDGE = 3
 */
void RSCounter::setPinATriggerType( uint8_t addr, uint8_t mode)
{
	uint8_t msg[RSMSGLEN];
	
	msg[0] ='c';            // sends instruction counter 			
	msg[1] ='t';             // sub: trigger
	msg[2] = mode;   
	msg[3] = 0x00;

	rsCom.writeMessage(addr,msg,RSMSGLEN); // send the complete message
}

/*!
 * \brief Reset the count.
 *
 *
 * \param[in] addr address of the board, 0x10 is the default board I2C address
 */
void RSCounter::resetCount( uint8_t addr )
{
	uint8_t msg[RSMSGLEN];
	
	msg[0] ='c';            // sends instruction counter 			
	msg[1] ='z';             // sub: 'z'ero the counter
	msg[2] = 0x00;   
	msg[3] = 0x00;

	rsCom.writeMessage(addr,msg,RSMSGLEN); // send the complete message
}
/*!
 * \brief Get the count
 *
 *
 * \param[in] addr address of the board, 0x10 is the default board I2C address
 * \return number of count since resetCount or start.
 */
int32_t RSCounter::getCount( uint8_t addr)
{
		int32_t res = 0;
	uint8_t msg[RSMSGLEN];
	uint8_t resp[RSRESPLEN];
	uint8_t hi[2]; // store the hi-part
	uint8_t cnt = 0;
	

	msg[0] ='c';            // sends instruction counter 			
	msg[1] ='h';             // sub: get hi value
	msg[2] = 0x00;   
	msg[3] = 0x00;

	rsCom.writeMessage(addr,msg,RSMSGLEN); // send the complete message
	
	resp[0] = 0;
	resp[1] = 0;
	resp[2] = 0;
	cnt= rsCom.readResponse(addr,&resp[0], RSRESPLEN);
	
	if ( cnt < 3)
	{
		hi[0] = 0;
		hi[1] = 0;
	}
	else
	{
		hi[0] = resp[1];
		hi[1] = resp[2];
	}
	
	msg[0] ='c';            // sends instruction counter 			
	msg[1] ='l';             // sub: get low value
	msg[2] = 0x00;   
	msg[3] = 0x00;

	rsCom.writeMessage(addr,msg,RSMSGLEN); // send the complete message
	
	resp[0] = 0;
	resp[1] = 0;
	resp[2] = 0;
	cnt= rsCom.readResponse(addr,&resp[0], RSRESPLEN);
	
	if ( cnt < 3)
	{
		res = 0;
	}
	else
	{	
	res = int32_t ((hi[0] << 24) | (hi[1] << 16) | (resp[1] << 8) | resp[2]); // big endian
	}
		
	return res;	
}

/*!
 * \class RSStepper
 *
 * \brief RSStepper allow a stepper motor on a board.
 *
 * Step motor who have a DIRECTION, STEP driver can be used.
 *
 * \author Stephane Rousseau
 * \date
 */
RSStepper::RSStepper()
{
}

/*!
 * \brief Set the pins of the counter.
 *
 *
 * \param[in] addr address of the board, 0x10 is the default board I2C address
 * \param[in] dirPin direction pin
 * \param[in] stepPin step pin
 */
void  RSStepper::setPins( uint8_t addr, uint8_t dirPin, uint8_t stepPin)
{
	uint8_t msg[RSMSGLEN];
	
	msg[0] ='m';            // sends instruction move 			
	msg[1] ='p';             // sub: 'p'ins
	msg[2] = dirPin;   
	msg[3] = stepPin;

	rsCom.writeMessage(addr,msg,RSMSGLEN); // send the complete message
}

/*!
 * \brief Set the linear acceleration profile.
 *
 *  If sp0 and spMax are the same, constant speed is used.
 *
 * \param[in] addr address of the board, 0x10 is the default board I2C address
 * \param[in] sp0 startSpeed default 800 step/s, max 9000 step/s
 * \param[in] spMax max speed default 1600 steps/s,  max 9000 step/s
 * \param[in] accel default 6400 step/s^2, max 65000 step/s
 * \param[in] decel default 6400 step/s^2, max 65000 step/s.
 */

uint8_t  RSStepper::setProfile( uint8_t addr, int16_t sp0, int16_t spMax, int16_t accel, int16_t decel)
{
	uint8_t res = RSBOARD_UNKNOWN_ERROR;
	uint8_t msg[RSMSGLEN];
	
	if ( sp0 > spMax )
	{
		res = RSBOARD_SP0MUSTBEEQUALORLOWERTHANSPMAX_ERROR;
	}
	else if ( accel > 0 && decel > 0)
	{
		msg[0] ='m';            // sends instruction move 			
		msg[1] ='z';             // spzero
		msg[2] =(uint8_t)((sp0 & 0xff00) >> 8);   
		msg[3] =(uint8_t) (sp0 & 0x00ff);
		rsCom.writeMessage(addr,msg,RSMSGLEN); // send the complete message

		msg[0] ='m';            // sends instruction move 			
		msg[1] ='m';             // spmax
		msg[2] =(uint8_t)((spMax & 0xff00) >> 8);   
		msg[3] =(uint8_t)(spMax & 0x00ff);
		rsCom.writeMessage(addr,msg,RSMSGLEN); // send the complete message
		
		msg[0] ='m';            // sends instruction move 			
		msg[1] ='a';             // low part of accel
		msg[2] =(uint8_t)((accel & 0x0000ff00) >> 8);   
		msg[3] =(uint8_t)(accel & 0x000000ff);
		rsCom.writeMessage(addr,msg,RSMSGLEN); // send the complete message

		msg[0] ='m';            // sends instruction move 			
		msg[1] ='d';             // low part of accel
		msg[2] =(uint8_t)((decel & 0x0000ff00) >> 8);   
		msg[3] =(uint8_t)(decel & 0x000000ff);
		rsCom.writeMessage(addr,msg,RSMSGLEN); // send the complete message
		
		res = RSBOARD_SUCCESS;
	
	}
	else
	{
		res = RSBOARD_ACCELANDDECELMUSTBEPOSITIVEGREATERTHANZERO;
	}
	
	return res;
}

/*!
 * \brief Move the stepper 
 *
 *  If steps is negative, move at reverse.
 *
 * \param[in] addr address of the board, 0x10 is the default board I2C address
 * \param[in] steps number of steps to move.
 */
void  RSStepper::move( uint8_t addr, int32_t steps)
{
	uint8_t msg[RSMSGLEN];
	
	msg[0] = 'm';           // sends instruction move 			
	msg[1] ='h';             // 16 bits high value of  
	msg[2] = (uint8_t)((steps & 0xff000000) >> 24); // hi  
	msg[3] = (uint8_t)((steps & 0x00ff0000) >> 16);
	
	rsCom.writeMessage(addr,msg,RSMSGLEN); // send the complete message
  
	msg[0] ='m';            // sends instruction move 			
	msg[1] ='l';             // 16 bits low value of steps
	msg[2] =(uint8_t)((steps & 0x0000ff00) >> 8);   
	msg[3] =(uint8_t)(steps & 0x000000ff);
	
	rsCom.writeMessage(addr,msg,RSMSGLEN); // send the complete message
}

/*!
 * \brief Get the number of steps remaining.
 *
 *
 * \param[in] addr address of the board, 0x10 is the default board I2C address
 * \returm number of steps remaining
 */
int32_t  RSStepper::getToGo( uint8_t addr)
{
	
	int32_t res = 0;
	uint8_t msg[RSMSGLEN];
	uint8_t resp[RSRESPLEN];
	uint8_t hi[2]; // store the hi-part
	uint8_t cnt = 0;
	

	msg[0] ='m';			// sends instruction move			
	msg[1] ='g'; 			// sub: get hi value
	msg[2] = 'h';   
	msg[3] = 0x00;

	rsCom.writeMessage(addr,msg,RSMSGLEN); // send the complete message
	
	resp[0] = 0;
	resp[1] = 0;
	resp[2] = 0;
	cnt= rsCom.readResponse(addr,&resp[0], RSRESPLEN);
	
	if ( cnt < 3)
	{
		hi[0] = 0;
		hi[1] = 0;
	}
	else
	{
		hi[0] = resp[1];
		hi[1] = resp[2];
	}
	
	msg[0] ='m';            // sends instruction move 			
	msg[1] ='g';             // sub: get low value
	msg[2] = 'l';   
	msg[3] = 0x00;
	
	rsCom.writeMessage(addr,msg,RSMSGLEN); // send the complete message
	
	resp[0] = 0;
	resp[1] = 0;
	resp[2] = 0;
	
	
	cnt= rsCom.readResponse(addr,&resp[0], RSRESPLEN);
	
	if ( cnt < 3)
	{
		res = -1;
	}
	else
	{	
		res = ((hi[0] << 24) | (hi[1] << 16) | (resp[1] << 8) | resp[2]); // big endian
	}
	return res;	
}




// Create one instance of the rsCom to be used by script
RSComChannel rsCom = RSComChannel();