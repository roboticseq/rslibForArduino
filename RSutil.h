/*
  RSutil.h - Robotic Sequencing utility library for Arduino & Wiring
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

#ifndef RSutil_h
#define RSutil_h

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
  #else
  #include "WProgram.h"
  #endif
  
uint8_t RSreadLine( uint8_t * buf, uint8_t maxLen, boolean echo);
void RSwriteLine( uint8_t * buf, uint8_t len);
int RSstrToInt( uint8_t * buf, uint8_t len);
String charHexToString( uint8_t ch);
#endif