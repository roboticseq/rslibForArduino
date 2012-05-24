/*
  RSutil.cpp - Robotic Sequencing utility library for Wiring & Arduino
  Copyright (c) 2012 Stephane Rousseau.  All right reserved.

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



#include "RSutil.h"

uint8_t RSreadLine( uint8_t * buf, uint8_t maxLen, boolean echo)
{
	char c; 
    boolean done = false;
	int i = 0;
	
	while(!done) {  
		if ( Serial.available()){
			c = Serial.read();
			if ( echo ){
				Serial.write(c);
			}
			if ( i >= (maxLen-1) || c == '\n' || c == '\r'){
				buf[i] = 0;
                done = true;
				break;
			}
			else {
				buf[i++] = c;
			}
		}
	}
	Serial.write("\r\n");
	
	return i;
}

void RSwriteLine( uint8_t * buf, uint8_t len)
{
	char c; 
	int i = 0;
	while(i < len) {  
		Serial.write(buf[i++]);
	}
	Serial.write("\r\n");
}

int RSstrToInt( uint8_t * buf, uint8_t len)
{
	char c; 
	int i = 0;
	int res = 0;
	boolean negative = false;
	for ( i = 0; i<len; i++){  
		c = buf[i];
		if ( c >= '0' && c <= '9'){
			res = res*10 + ( c - '0');
		}
		else if ( c == '-' )
		{
			negative = !negative;
		}
	}
        if ( negative ){
          res = -res;
        }
	return res;
}

// return a string from an hex number including the leading zero 
String charHexToString( uint8_t ch)
{
	if ( ch < 0x10)
	{
		return String("0"+String(ch,HEX));
	}
	else
	{
		return String(ch,HEX);
	}
}