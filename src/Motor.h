/*
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
