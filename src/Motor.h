/*
Scoperta Junior robot platform source
Copyright (C) 2013 Discover Scoperta Ltd.

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.

Author: Kevin Loney (kevin.loney@brainsinjars.com)
*/

#ifndef __MOTOR_H__
#define __MOTOR_H__

#if defined(ARDUINO) && ARDUINO >= 100
	#include <Arduino.h>
#else
	#include <WProgram.h>
	#include <pins_arduino.h>
#endif

#include <avr/io.h>

class Motor {
private:
	int _speed;

	int _enable;	// Enable pin
	uint8_t _bit_in1, _bit_in2;		// IN* bit masks
	uint8_t _port_in1, _port_in2;	// IN* ports
	volatile uint8_t *_reg_in1;		// IN1 register
	volatile uint8_t *_reg_in2;		// IN2 register

public:
	// Constructor
	Motor(int enable, int in1, int in2);

	void speed(int);	// Set motor speed
	int speed(void);	// Get motor speed
};

#endif
