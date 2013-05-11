//#include <IRremote.h>
#include <NewPing.h>
#include <Servo.h>
#include "Motor.h"

// Operation mode definitions
#define MODE_AUTO 0
#define MODE_LINE 1
#define MODE_REMOTE 2

// Macros for handling motor control
#define turnLeft(speed)		motorControl((speed), -(speed))
#define turnRight(speed)	motorControl(-(speed), (speed))
#define forward(speed)		motorControl((speed), (speed))
#define reverse(speed)		motorControl(-(speed), -(speed))
#define stop()				motorControl(0, 0)

// IR Remote control
const int IR_PIN = 0; // IR remote input pin

// Motor control pins
const int ENA_PIN = 6; //grey <- RIGHT
const int ENB_PIN = 3; //orange <-LEFT

const int AIN1_PIN = 4; //purple
const int AIN2_PIN = 7; //blue
const int BIN1_PIN = 8; //green
const int BIN2_PIN = 9; //yellow

// Ultrasonic steering 
const int SERVO_PIN = 5; //Servo
const int TRIGGER_PIN = 13;  // Yellow -Arduino pin tied to trigger pin on ping sensor.
const int ECHO_PIN = 12;  // Orange - Arduino pin tied to echo pin on ping sensor.

// Line following
const int LINE_L_PIN = 0; // Left - Red 
const int LINE_C_PIN = 1; // Center - White
const int LINE_R_PIN = 2; // Right - Blue

const int MAX_DISTANCE = 120; // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
const int FUZZ_DISTANCE = 30;

const int SERVO_CENTER = 84;
const int MAX_SERVO_SWEEP = 70;
const int TURN_DISTANCE = 15;	// How far away from an object (in centimeters) should we be before we start turning

const int BOT_WIDTH = 18; // Width of the bot in centimeters

const int MAX_TURN_TIME = 800;
const int MIN_TURN_TIME = 200;

const int MAX_SPEED = 200; // 0-255: Maximum wheel drive speed
const int MIN_SPEED = 100; // 0-255: Minimum wheel drive speed

const int LINE_THRESHOLD = 511;
const int LINE_RAMP_TIME = 32;
boolean LINE_IS_LIGHT = false;	// false for dark lines/true for light lines

int mode = MODE_LINE;

//IRrecv ir(IR_PIN);
//decode_results ir_data;

Servo servo;        // servo objects
NewPing srf06(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

Motor rmotor(ENA_PIN, AIN1_PIN, AIN2_PIN);
Motor lmotor(ENB_PIN, BIN2_PIN, BIN1_PIN);

void setup() {
  Serial.begin(9600);
  
  //ir.enableIRIn();
  
  // Center the servo
  servo.attach(SERVO_PIN);
  servo.write(SERVO_CENTER); // set servo to center pois
}
 
void loop() {
    switch(mode) {
        case MODE_AUTO: modeAutonomous(); break;
        case MODE_LINE: modeLineFollow(); break;
        case MODE_REMOTE: modeRemoteControl(); break;
        default:
			mode = MODE_AUTO;
			break;
    }

/*
    if(ir.decode(&ir_data)) {
        ir.resume();
    }
*/
}

int turnRandom = 0;
unsigned long turnAttempts = 0;

void modeAutonomous() {
    int distance = srf06.ping();
    if(distance > 0) {
        int speed = map(distance, 0, MAX_DISTANCE, MIN_SPEED, MAX_SPEED);

        if(distance > TURN_DISTANCE * US_ROUNDTRIP_CM) {
            forward(speed);
        }
        else {
            reverse(MAX_SPEED);
            delay(50);
            stop();

            int theta = constrain((int)floor(180.0 * atan2((US_ROUNDTRIP_CM * BOT_WIDTH) >> 1, distance) / M_PI + 0.5), 0, MAX_SERVO_SWEEP);

            servo.write(SERVO_CENTER + theta);
            delay(100);
            int lDist = srf06.ping();

            servo.write(SERVO_CENTER - theta);
            delay(100);
            int rDist = srf06.ping();

            if(abs(lDist - rDist) < FUZZ_DISTANCE) {
                if(turnRandom == 0) {
                    turnRandom = ((random() & 1) << 1) - 1;
                }
                if(turnRandom < 0) {
                    turnLeft(MAX_SPEED);
                }
                else {
                    turnRight(MAX_SPEED);
                }
            }
            else if(lDist > rDist || lDist == 0) {
                turnLeft(MAX_SPEED);
            }
            else {
               turnRight(MAX_SPEED);
            }
            delay(map(turnAttempts, 0, 10, MIN_TURN_TIME, MAX_TURN_TIME));
            ++turnAttempts;

            servo.write(SERVO_CENTER);
        }
    }
    else {
        turnRandom = 0;
        turnAttempts = 0;
        forward(MAX_SPEED);
    }
}

int lineSpeed = 0;
int turnDirection = 0;
int lineLastState = 0;

void modeLineFollow() {
	int state = 0;
	
	if(analogRead(LINE_L_PIN) > LINE_THRESHOLD) { state |= 0x1; }
	if(analogRead(LINE_C_PIN) > LINE_THRESHOLD) { state |= 0x2; }
	if(analogRead(LINE_R_PIN) > LINE_THRESHOLD) { state |= 0x4; }

	// Flip the state when looking for a light line on a dark background
	if(LINE_IS_LIGHT) {
		state = (~state) & 0x7;
	}

	switch(state) {
		case 0x0: {
				if(!turnDirection) {
					Serial.print(lineLastState);
					switch(lineLastState) {
						case 0x1:
						case 0x3:
							turnDirection = -1;
							turnLeft(MAX_SPEED);
							break;
						case 0x2:
							turnDirection = 2;
							reverse(MAX_SPEED);
							break;
						case 0x4:
						case 0x6:
							turnDirection = -1;
							turnRight(MAX_SPEED);
							break;
						default:
							if(turnDirection == 0) {
								turnDirection = (random() & 2) - 1;
								motorControl(turnDirection * MIN_SPEED, -turnDirection * MIN_SPEED);
							}
							break;
					}
				}
			} return;
		case 0x2:
		case 0x5:
		case 0x7:
			forward(map(lineSpeed, 0, LINE_RAMP_TIME, MIN_SPEED, MAX_SPEED));
			lineSpeed = min(++lineSpeed, LINE_RAMP_TIME);
			break;
		case 0x1:
		case 0x3:
			lineSpeed = 0;
			motorControl(MAX_SPEED, MIN_SPEED);
			break;
		case 0x4:
		case 0x6:
			lineSpeed = 0;
			motorControl(MIN_SPEED, MAX_SPEED);
			break;
		default: break;
	}
	if(state) {
		turnDirection = 0;
	}
	lineLastState = state;
}

void modeRemoteControl() {

}

void motorControl(int rSpeed, int lSpeed) {
	rmotor.speed(rSpeed);
	lmotor.speed(lSpeed);
}
