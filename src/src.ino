#include <NewPing.h>
#include <Servo.h>
 
const int ENA_PIN = 6; //grey <- RIGHT
const int ENB_PIN = 3; //orange <-LEFT

const int AIN1_PIN = 10; //purple
const int AIN2_PIN = 5; //blue
const int BIN1_PIN = 2; //green
const int BIN2_PIN = 4; //yellow

const int SERVO_PIN = 8; //Servo

const int TRIGGER_PIN = 12;  // Yellow -Arduino pin tied to trigger pin on ping sensor.
const int ECHO_PIN = 11;  // Orange - Arduino pin tied to echo pin on ping sensor.

const int MAX_DISTANCE = 90; // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
const int SERVO_CENTER = 84;
const int MAX_SERVO_SWEEP = 70;

#define SAMPLES 20

#define MODE_START 0
#define MODE_AUTO 1
#define MODE_REMOTE 2

int mode = MODE_START;

Servo servo;        // servo objects
NewPing srf06(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

 
void setup() {
  Serial.begin(9600);

  // Configure h0bridge cotrol pins
  digitalWrite(ENA_PIN, LOW); //Enable Right Motor
  digitalWrite(ENB_PIN, LOW); //Enable Left Motor
  analogWrite(AIN2_PIN, LOW); //<- On for reverse
  analogWrite(AIN1_PIN, LOW); //<- On for Forward
  analogWrite(BIN2_PIN, LOW); //<- On for reverse
  analogWrite(BIN1_PIN, LOW); //<- On for Forward    

  servo.attach(SERVO_PIN);
  servo.write(SERVO_CENTER); // set servo to center pois
}
 
void loop() {
    switch(mode) {
        case MODE_START:
          return;
        case MODE_AUTO: break;
        case MODE_REMOTE: break;
        default: break;
    }
    return;

/*
  fwdDistance =  srf06.ping_cm();
//  delay(100);
  if (fwdDistance > 30 || fwdDistance == 0)
  {
    forward();
    Serial.println("forward");
  }
  else
  {
    stop();
    MrServo.write(ServoCenter + ServoOffset);  // move servo left
    delay(500);
    leftDistance = srf06.ping_cm();
    delay(100);
    MrServo.write(ServoCenter - ServoOffset);    // move servo right
    delay(500);
    rightDistance = srf06.ping_cm();
    delay(100);
    MrServo.write(ServoCenter);    // move servo to center
    delay(50);
    leftDistance = leftDistance == 0?0x7FFF:leftDistance;
    rightDistance = rightDistance == 0?0x7FFF:rightDistance;
    compareDistance();
 
  }     
*/
}
 
void compareDistance()
{
    /*
  Serial.print(leftDistance);
  Serial.print(" ");
  Serial.println(rightDistance);
  if (leftDistance>rightDistance) //if left is less obstructed
  {
    turnleft();
    Serial.println("left");
    delay(800);
  }
  else if (rightDistance>leftDistance) //if right is less obstructed
  {
    turnright();
    Serial.println("right");
    
    delay(800);
  }
  else //if (rightDistance = leftDistance)
  {
    turnleft();
     Serial.println("no good option, go left");
    delay(1500);
  }
  */
}

void turnLeft(int speed) {
    motorControl(speed, -speed);
}

void turnRight(int speed) {
    motorControl(-speed, speed);
}

void forward(int speed) {
    motorControl(speed, speed);
}

void halt() {
    motorControl(0, 0);
}

void motorControl(int rSpeed, int lSpeed) {
    rSpeed = constrain(rSpeed, -255, 255);
    lSpeed = constrain(lSpeed, -255, 255);

    digitalWrite(ENA_PIN, rSpeed == 0 ? LOW : HIGH);
    digitalWrite(ENB_PIN, lSpeed == 0 ? LOW : HIGH);

    if(rSpeed > 0) {
        analogWrite(AIN2_PIN, LOW); //<- On for reverse
        analogWrite(AIN1_PIN, rSpeed); //<- On for Forward
    }
    else {
        analogWrite(AIN2_PIN, -rSpeed); //<- On for reverse
        analogWrite(AIN1_PIN, LOW); //<- On for Forward
    }

    if(lSpeed > 0) {
        analogWrite(BIN2_PIN, lSpeed); //<- On for reverse
        analogWrite(BIN1_PIN, LOW); //<- On for Forward    
    }
    else {
        analogWrite(BIN2_PIN, LOW); //<- On for reverse
        analogWrite(BIN1_PIN, -lSpeed); //<- On for Forward    
    }
}
