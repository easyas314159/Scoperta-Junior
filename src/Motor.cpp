/*
Author: Kevin Loney (kevin.loney@brainsinjars.com)
*/
#include "Motor.h"

// Macros for quickly setting and clearing digital pins
#define SET(port, mask) ((port) |= (mask))
#define CLR(port, mask) ((port) &= ~(mask))

Motor::Motor(int enable, int in1, int in2) {
	_enable = enable;

	// Make sure the the direction pins are both in output mode
	pinMode(in1, OUTPUT);
	pinMode(in2, OUTPUT);

	// Retrieve the pin bit masks
	_bit_in1 = digitalPinToBitMask(in1);
	_bit_in2 = digitalPinToBitMask(in2);

	// Retrieve the pin port numbers
	_port_in1 = digitalPinToPort(in1);
	_port_in2 = digitalPinToPort(in2);

	// Retrieve the port registers
	_reg_in1 = portOutputRegister(_port_in1);
	_reg_in2 = portOutputRegister(_port_in2);

	// Start with the motors stopped
	speed(0);
}

void Motor::speed(int speed) {
	// Contrain the motor speed to a reasonable range
	_speed = constrain(speed, -255, 255);
	analogWrite(_enable, abs(speed));

	// Set the direction pins appropriately
	if(_speed < 0) {
		CLR(*_reg_in1, _bit_in1);
		SET(*_reg_in2, _bit_in2);
	}
	else {
		CLR(*_reg_in2, _bit_in2);
		SET(*_reg_in1, _bit_in1);
	}
}

// Get the motors current speed
int Motor::speed(void) {
	return _speed;
}
