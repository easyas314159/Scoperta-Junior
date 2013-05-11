/*
Author: Kevin Loney (kevin.loney@brainsinjars.com)
*/

//#include <IRremote.h>
#include <NewPing.h>
#include <Servo.h>

#include "Motor.h"

// Operation mode definitions
#define MODE_AUTO 0		// Autonomous driving mode
#define MODE_LINE 1		// Line following mode
#define MODE_REMOTE 2	// IR remote control mode

// Macros for handling motor control
#define turnLeft(speed)		motorControl((speed), -(speed))
#define turnRight(speed)	motorControl(-(speed), (speed))
#define forward(speed)		motorControl((speed), (speed))
#define reverse(speed)		motorControl(-(speed), -(speed))
#define stop()				motorControl(0, 0)

// IR Remote control
const int IR_PIN = 0; // IR remote input pin

// Motor control pins
// Right
const int ENA_PIN = 6;	// grey
const int AIN1_PIN = 4;	// purple
const int AIN2_PIN = 7;	// blue

// Left
const int ENB_PIN = 3;	// orange
const int BIN1_PIN = 8;	// green
const int BIN2_PIN = 9;	// yellow

// Ultrasonic steering 
const int SERVO_PIN = 5;	// Servo
const int TRIGGER_PIN = 13;	// Yellow -Arduino pin tied to trigger pin on ping sensor.
const int ECHO_PIN = 12;	// Orange - Arduino pin tied to echo pin on ping sensor.

// Line following
const int LINE_L_PIN = 0;	// Left - Red 
const int LINE_C_PIN = 1;	// Center - White
const int LINE_R_PIN = 2;	// Right - Blue

// General configuration
const int MAX_SPEED = 200; // 0-255: Maximum wheel drive speed
const int MIN_SPEED = 100; // 0-255: Minimum wheel drive speed

// Autonomous mode configuration
const int MAX_DISTANCE = 120;	// Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
const int TURN_DISTANCE = 15;	// How far away from an object (in centimeters) should we be before we start turning
const int FUZZ_DISTANCE = 30;	// How different (in microseconds) do the left and right ping times have to be to be considered different

const int SERVO_CENTER = 84;	// The servo angle that will be considered as center
const int MAX_SERVO_SWEEP = 70;	// The maximum angle the head will turn off the SERVO_CENTER

const int BOT_WIDTH = 18;		// Width of the bot in centimeters

const int MAX_TURN_TIME = 800;	// Maximum time to turn for while avoiding obstacles
const int MIN_TURN_TIME = 200;	// Minimum time to turn for while avoiding obstacles

// Line mode configuration
boolean LINE_IS_LIGHT = false;	// false for dark lines/true for light lines
const int LINE_THRESHOLD = 511;	// The threshold for detecting a line
const int LINE_RAMP_TIME = 32;	// How quickly 

// Current mode of operation
int mode = MODE_LINE;

// IR receiver init
//IRrecv ir(IR_PIN);
//decode_results ir_data;

// Ultrasonics init
Servo servo;        // servo objects
NewPing srf06(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

// Motor init
Motor rmotor(ENA_PIN, AIN1_PIN, AIN2_PIN);
Motor lmotor(ENB_PIN, BIN2_PIN, BIN1_PIN);

void setup() {
	Serial.begin(9600);

	// Enable IR input
	//ir.enableIRIn();

	// Center the servo
	servo.attach(SERVO_PIN);
	servo.write(SERVO_CENTER); // set servo to center position
}

void loop() {
    switch(mode) {
        case MODE_AUTO: modeAutonomous(); break;
        case MODE_LINE: modeLineFollow(); break;
        case MODE_REMOTE: modeRemoteControl(); break;
        default:
			mode = MODE_AUTO;	// Invalid modes default to autonomous
			break;
    }

/*
    if(ir.decode(&ir_data)) {
        ir.resume();
    }
*/
}

// Autonomous mode variables
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
			// TODO: This should to be rewritten to be non-blocking
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

// Line mode variables
int lineSpeed = 0;
int turnDirection = 0;
int lineLastState = 0;

void modeLineFollow() {
	int state = 0;
	
	// Determine current line state
	if(analogRead(LINE_L_PIN) > LINE_THRESHOLD) { state |= 0x1; }
	if(analogRead(LINE_C_PIN) > LINE_THRESHOLD) { state |= 0x2; }
	if(analogRead(LINE_R_PIN) > LINE_THRESHOLD) { state |= 0x4; }

	// Flip the state when looking for a light line on a dark background
	if(LINE_IS_LIGHT) {
		state = (~state) & 0x7;
	}

	// To tired to write proper description of what is going on here
	// MAGIC!
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
	// TODO: Implement me!
}

void motorControl(int rSpeed, int lSpeed) {
	rmotor.speed(rSpeed);
	lmotor.speed(lSpeed);
}
