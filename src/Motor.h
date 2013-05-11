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
	int _enable;

	int _min_speed;
	int _max_speed;

	uint8_t _bit_in1, _bit_in2;
	uint8_t _port_in1, _port_in2;
	volatile uint8_t *_reg_in1, *_reg_in2;

public:
	Motor(int, int, int);

	void speed(int);
	int speed(void);
};

#endif
