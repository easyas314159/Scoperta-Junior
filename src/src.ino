#include <NewPing.h>
#include <Servo.h>
 
const int ENA_PIN = 6; //grey <- RIGHT
const int ENB_PIN = 3; //orange <-LEFT

const int AIN1_PIN = 10; //purple
const int AIN2_PIN = 5; //blue
const int BIN1_PIN = 2; //green
const int BIN2_PIN = 4; //yellow

const int servopin = 8; //Servo

const int TRIGGER_PIN = 12;  // Yellow -Arduino pin tied to trigger pin on ping sensor.
const int ECHO_PIN = 11;  // Orange - Arduino pin tied to echo pin on ping sensor.

const int MAX_DISTANCE = 200; // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
const int ServoCenter = 93;
const int ServoOffset = 70;
const int mtrSpeed = 255;

NewPing srf06(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
 
int leftDistance, rightDistance, fwdDistance;
 
Servo MrServo;        // servo objects
 
void setup() {
  Serial.begin(9600);

  digitalWrite(ENA_PIN, LOW); //Enable Right Motor
  digitalWrite(ENB_PIN, LOW); //Enable Left Motor
  analogWrite(AIN2_PIN, LOW); //<- On for reverse
  analogWrite(AIN1_PIN, LOW); //<- On for Forward
  analogWrite(BIN2_PIN, LOW); //<- On for reverse
  analogWrite(BIN1_PIN, LOW); //<- On for Forward    
 
  MrServo.attach(servopin);      
  MrServo.write(ServoCenter) ; // set servo to center pois
  delay(150);
}
 
void loop() {
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
}
 
void compareDistance()
{
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
}
 
void turnleft()
{
    digitalWrite(ENA_PIN, HIGH); //Enable Right Motor
    digitalWrite(ENB_PIN, HIGH); //Enable Left Motor
    analogWrite(AIN2_PIN, LOW); //<- On for reverse
    analogWrite(AIN1_PIN, mtrSpeed); //<- On for Forward
    analogWrite(BIN2_PIN, LOW); //<- On for reverse
    analogWrite(BIN1_PIN, mtrSpeed); //<- On for Forward    
}
 
void turnright()
{
    digitalWrite(ENA_PIN, HIGH); //Enable Right Motor
    digitalWrite(ENB_PIN, HIGH); //Enable Left Motor
    analogWrite(AIN2_PIN, mtrSpeed); //<- On for reverse
    analogWrite(AIN1_PIN, LOW); //<- On for Forward
    analogWrite(BIN2_PIN, mtrSpeed); //<- On for reverse
    analogWrite(BIN1_PIN, LOW); //<- On for Forward    
}
 
void forward()
{
    digitalWrite(ENA_PIN, HIGH); //Enable Right Motor
    digitalWrite(ENB_PIN, HIGH); //Enable Left Motor
    analogWrite(AIN2_PIN, LOW); //<- On for reverse
    analogWrite(AIN1_PIN, mtrSpeed); //<- On for Forward
    analogWrite(BIN2_PIN, mtrSpeed); //<- On for reverse
    analogWrite(BIN1_PIN, LOW); //<- On for Forward    
}
 
void stop()
{
    digitalWrite(ENA_PIN, LOW); //Enable Right Motor
    digitalWrite(ENB_PIN, LOW); //Enable Left Motor
    analogWrite(AIN2_PIN, LOW); //<- On for reverse
    analogWrite(AIN1_PIN, LOW); //<- On for Forward
    analogWrite(BIN2_PIN, LOW); //<- On for reverse
    analogWrite(BIN1_PIN, LOW); //<- On for Forward    
}
