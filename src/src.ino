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

const int MAX_DISTANCE = 120; // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
const int TURN_DISTANCE = 15 * US_ROUNDTRIP_CM;
const int FUZZ_DISTANCE = 30;

const int SERVO_CENTER = 84;
const int MAX_SERVO_SWEEP = 70;

const int BOT_WIDTH = 18; // Width of the bot in centimeters

const int MAX_TURN_TIME = 800;
const int MIN_TURN_TIME = 200;

const int MAX_SPEED = 200; // 0-255: Maximum wheel drive speed
const int MIN_SPEED = 100; // 0-255: Minimum wheel drive speed


int mode = MODE_AUTO;

//IRrecv ir(IR_PIN);
//decode_results ir_data;

Servo servo;        // servo objects
NewPing srf06(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

Motor rmotor(ENA_PIN, AIN1_PIN, AIN2_PIN);
Motor lmotor(ENB_PIN, BIN2_PIN, BIN1_PIN);

void setup() {
  Serial.begin(9600);
  
  // Initial motor configuration
  pinMode(AIN1_PIN, OUTPUT);
  pinMode(AIN2_PIN, OUTPUT);
  pinMode(BIN1_PIN, OUTPUT);
  pinMode(BIN2_PIN, OUTPUT);
  stop();

  //ir.enableIRIn();
  
  // Center the servo
  servo.attach(SERVO_PIN);
  servo.write(SERVO_CENTER); // set servo to center pois
}
 
void loop() {
    switch(mode) {
        case MODE_AUTO: modeAutonomous(); break;
        case MODE_REMOTE: break;
        default: break;
    }

/*
    if(ir.decode(&ir_data)) {
        ir.resume();
    }
*/

    return;
}

int turnRandom = 0;
unsigned long turnAttempts = 0;

unsigned long modeDistance = 0;
unsigned long nextDistance = 0;

void modeAutonomous() {
    int distance = srf06.ping();
    if(distance > 0) {
        int speed = map(distance, 0, MAX_DISTANCE, MIN_SPEED, MAX_SPEED);

        if(distance > TURN_DISTANCE) {
            forward(speed);
        }
        else {
            reverse(MAX_SPEED);
            delay(50);
            stop();

            int theta = constrain((int)floor(180.0 * atan2((US_ROUNDTRIP_CM * BOT_WIDTH) >> 1, distance) / M_PI + 0.5), 0, MAX_SERVO_SWEEP);

            servo.write(SERVO_CENTER + theta);
            delay(100);
            int lDist = srf06.ping_cm();

            servo.write(SERVO_CENTER - theta);
            delay(100);
            int rDist = srf06.ping_cm();

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




void motorControl(int rSpeed, int lSpeed) {
	rmotor.speed(rSpeed);
	lmotor.speed(lSpeed);
}
