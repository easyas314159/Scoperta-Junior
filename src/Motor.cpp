#include "Motor.h"

#define SET(port, mask) ((port) |= (mask))
#define CLR(port, mask) ((port) &= ~(mask))

Motor::Motor(int enable, int in1, int in2) {
	_enable = enable;

	pinMode(in1, OUTPUT);
	pinMode(in2, OUTPUT);

	_bit_in1 = digitalPinToBitMask(in1);
	_bit_in2 = digitalPinToBitMask(in2);

	_port_in1 = digitalPinToPort(in1);
	_port_in2 = digitalPinToPort(in2);

	_reg_in1 = portOutputRegister(_port_in1);
	_reg_in2 = portOutputRegister(_port_in2);

	speed(0);
}

void Motor::speed(int speed) {
	_speed = constrain(speed, -255, 255);
	analogWrite(_enable, abs(speed));

	if(_speed < 0) {
		CLR(*_reg_in1, _bit_in1);
		SET(*_reg_in2, _bit_in2);
	}
	else {
		CLR(*_reg_in2, _bit_in2);
		SET(*_reg_in1, _bit_in1);
	}
}

int Motor::speed(void) {
	return _speed;
}
