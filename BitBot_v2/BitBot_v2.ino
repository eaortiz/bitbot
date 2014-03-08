#include <QTRSensors.h>
#include <Timers.h>
#include <Servo.h>

#define TIME_BACKWARDS 500
#define TURN_TIME 250
#define PUSHER_TIME 500
#define RIGHT 1
#define LEFT 0
#define MAX_TAPE_SENSOR_VAL 1023
#define RIGHT_MAX_SPEED 125
#define LEFT_MAX_SPEED 200
#define DELAY 500
#define TAPE_HIGH 10
#define TAPE_LOW 8
#define TIME_TO_MIDDLE 5000
#define TIME_TO_90 500
#define TIME_TO_180 1000
#define TIME_INTERVAL 2500
#define WALLDISTANCE 40
#define HOMING_INTERVAL 1000
#define STRAIGHTDELAYTIME 100

/*---------------- Module Level Variables ---------------------------*/
//analog
#define distanceTrigger 16
#define distanceEcho 17
#define centerSensorInput 7
#define serverSensorInput 11
#define frontTapeSensorInput 18
#define backTapeSensorInput 19

//digital
#define pusherEnable 6
#define rightWheelToggle 2
#define leftWheelToggle 4
#define rightMotorPWM 3
#define leftMotorPWM 5

//This isn't real
#define backRightBumperInput 6
#define backLeftBumperInput 7
//END

#define pusherToggle 8
#define threeCoinDumpOut 9
#define fiveCoinDumpOut 10
#define leftBumperInput 15
#define rightBumperInput 13

//states
int state;
//#define GET_DIRECTION 0 //going around looking for light
#define GO_TO_MIDDLE    1
//#define GO_TO_MIDDLE_2 3
#define AT_MIDDLE        0
#define LOOKING 2
#define APPROACH_SERVER 4
//#define GO_TO_SERVER 1 //going straight until server is hit
//#define REVERSE 2 //align 
//#define FORWARD 4 //reach tape
#define ALIGN_WITH_TAPE 5 //turn forward
#define GET_3_COINS 6
#define GET_5_COINS 7
#define TURNING 8
#define TURN_BACK 9
#define GOING_TO_CENTER 10
#define TURNING_TO_3 11
#define TURNING_TO_5 12
#define GOING_TO_3 13
#define GOING_TO_5 14
#define GOING_TO_8 15
#define GO_BACK 16
#define DUMPING 17
#define APPROACH_BEACON 18

//variables
int bumpedRightOrLeft = RIGHT; //right or left one is three
int coinsGotten = 0;
int coinsWanted = 0;
int pushes = 0;
boolean hasDumped = false;
int linesSensed = 0;
int curr_tape_sensor_values[2];
char *sequence_of_tape_sensor_changes; //A front on, B front off, C back on, D back off
byte byteRead;
int sideToAlign = RIGHT;
float TURN_SPEED = .5;

//for collision logic 
int nextLeft;
int nextRight;
int nextRightBack;
int nextLeftBack;

//Servos
Servo threeCoinDump;
Servo fiveCoinDump;  

void setup() { 
  Serial.begin(9600);
  Serial.println("Starting MEbot...");

  //pins
  pinMode(distanceTrigger,OUTPUT);
  pinMode(distanceEcho, INPUT);
  pinMode(centerSensorInput, INPUT);
  pinMode(serverSensorInput, INPUT);
  pinMode(frontTapeSensorInput, INPUT);
  pinMode(backTapeSensorInput, INPUT);
  pinMode(rightBumperInput, INPUT);
  pinMode(leftBumperInput, INPUT);
  pinMode(pusherToggle, OUTPUT);
  pinMode(rightWheelToggle, OUTPUT);
  pinMode(leftWheelToggle, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);
  pinMode(threeCoinDumpOut, OUTPUT);
  pinMode(fiveCoinDumpOut, OUTPUT);
  pinMode(pusherToggle, OUTPUT);
  pinMode(pusherEnable, OUTPUT);
  digitalWrite(pusherToggle,LOW);
  digitalWrite(pusherEnable,HIGH);

  //wheels
 digitalWrite(rightWheelToggle, LOW);
 digitalWrite(leftWheelToggle, HIGH);
  
  //servos
  threeCoinDump.attach(threeCoinDumpOut);
  fiveCoinDump.attach(fiveCoinDumpOut);
  threeCoinDump.write(0);
  fiveCoinDump.write(0);
  
  //collision logic
  nextLeft = digitalRead(leftBumperInput);
  nextRight = digitalRead(rightBumperInput);
  nextLeftBack = digitalRead(backLeftBumperInput);
  nextRightBack = digitalRead(backRightBumperInput);
  
  //pusher
  digitalWrite(pusherEnable, LOW);

  TMRArd_ClearTimerExpired(0);
  coinsGotten = 0
  coinsWanted = 8

}

void loop() {

  collect();
}
    

boolean nearWall() {
  return (getDistance() < WALLDISTANCE);
}

void lookAroundRight() { 
        adjustMotorSpeed(-1 * (float) RIGHT_MAX_SPEED * TURN_SPEED, (float) LEFT_MAX_SPEED * TURN_SPEED);
}

void lookAroundLeft() { 
        adjustMotorSpeed((float) RIGHT_MAX_SPEED * TURN_SPEED, -1 * (float) LEFT_MAX_SPEED * TURN_SPEED);
}

boolean serverLightSensed() { 
   unsigned long input;
   unsigned long inputtwo;
   unsigned long inputsum;
   input = pulseIn(serverSensorInput,LOW);
   inputtwo = pulseIn(serverSensorInput,HIGH);
   inputsum = input+inputtwo;
   return (inputsum < 1200 && inputsum > 1100);
}

boolean rightBumperHit() { 
      int currValue = digitalRead(rightBumperInput);
      if (!(currValue == nextRight)) {
          nextRight = currValue;
          Serial.println("right");
          return true;
      }
}

boolean leftBumperHit() {
        int currValue = digitalRead(leftBumperInput);
	if (!(currValue == nextLeft)) {
          nextLeft = currValue;
         Serial.println("left");
          return true;
	}
}

boolean rightBackBumperHit() {
        int currValue = digitalRead(backRightBumperInput);
	if (!(currValue == nextRightBack)) {
          nextRightBack = currValue;
          Serial.println("right back");
          return true;
	}
}

boolean leftBackBumperHit() { 
        int currValue = digitalRead(backLeftBumperInput);
	if (!(currValue == nextLeftBack)) {
          nextLeftBack = currValue;
          Serial.println("left back");
          return true;
	}
}

void goBackwards() {
  adjustMotorSpeed(-1 * RIGHT_MAX_SPEED/2, -5 * LEFT_MAX_SPEED/8);
  delay(STRAIGHTDELAYTIME);
  adjustMotorSpeed(-1 * RIGHT_MAX_SPEED/2, -1 * LEFT_MAX_SPEED/2);
}

void goForward() { 
  adjustMotorSpeed(RIGHT_MAX_SPEED/2, LEFT_MAX_SPEED/8*5);
  delay(STRAIGHTDELAYTIME);
  adjustMotorSpeed(RIGHT_MAX_SPEED/2,LEFT_MAX_SPEED/2);
}

void stopMotor() { 
  adjustMotorSpeed(0,0);
}

void alignWithTape() {
	if (sideToAlign == LEFT) { 
		adjustMotorSpeed(RIGHT_MAX_SPEED/4, -1 * LEFT_MAX_SPEED/4);
	}
	if (sideToAlign == RIGHT) { 
		adjustMotorSpeed(-1 * RIGHT_MAX_SPEED/4, LEFT_MAX_SPEED/4);
	}
}

void getThreeCoins() { 
	if (hasDumped) coinsWanted = 11;
	else coinsWanted = 3;
	TMRArd_InitTimer(0, PUSHER_TIME); 
}

void getFiveCoins() { 
	if (hasDumped) coinsWanted = 16;
	else coinsWanted = 8;
	TMRArd_InitTimer(0, PUSHER_TIME); 
}

void collect() { 
  if (coinsGotten < coinsWanted) { 
    pushAlgorithmButton();
    TMRArd_InitTimer(0, PUSHER_TIME);
  }
}

void pushAlgorithmButton() { 
	push();
	pushes += 1;
	if (pushes == coinsGotten) { 
		coinsGotten +=1;
	}

}

void push() {
  digitalWrite(pusherEnable, HIGH);
    digitalWrite(pusherToggle, HIGH);
    delay(PUSHER_TIME);
    digitalWrite(pusherToggle, LOW);
    digitalWrite(pusherEnable, LOW);
}

boolean doneCollecting() { 
	return coinsGotten == coinsWanted;
}

boolean centerSensorOn() { 
	unsigned long input = pulseIn(centerSensorInput, HIGH, 600);
        unsigned long input2 = pulseIn(centerSensorInput,LOW,600);
        unsigned long inputsum = input+input2;
	return (inputsum < 300 && inputsum > 350);
}

unsigned char TestTimerExpired(void) {
  unsigned char value = (unsigned char)TMRArd_IsTimerExpired(0);
  if (value == TMRArd_EXPIRED) {
    TMRArd_ClearTimerExpired(0);
  }
  return value;
}
//This function sweeps the servo searching the area for an object within range
void unloadThreeDumpServo() {

	Serial.println("Unloading...");
	for(int pos = 180; pos>=90; pos -= 1)     // goes from 180 degrees to 0 degrees 
	{                                
	  threeCoinDump.write(pos);              // tell servo to go to position in variable 'pos' 
	  delayMicroseconds(DELAY);
    }
	for(int pos = 90; pos < 180; pos += 1)  // goes from 0 degrees to 180 degrees 
	{                                  // in steps of 1 degree 
	  threeCoinDump.write(pos);              // tell servo to go to position in variable 'pos'
	  delayMicroseconds(DELAY);
	}    
}

void unloadFiveDumpServo() {

	Serial.println("Unloading...");
	for(int pos = 180; pos >= 90; pos -= 1)     // goes from 180 degrees to 0 degrees 
	{                                
	  fiveCoinDump.write(pos);              // tell servo to go to position in variable 'pos' 
	  delayMicroseconds(DELAY);
    }
	for(int pos = 90; pos < 180; pos += 1)  // goes from 0 degrees to 180 degrees 
	{                                  // in steps of 1 degree 
	  fiveCoinDump.write(pos);              // tell servo to go to position in variable 'pos'
	  delayMicroseconds(DELAY);
	}    
}

void adjustMotorSpeed(int rightSpeed, int leftSpeed)
{
  if (rightSpeed < 0) {
    digitalWrite(rightWheelToggle, HIGH);
  } else {
    digitalWrite(rightWheelToggle, LOW);
  }
  
  if (leftSpeed < 0) {
    digitalWrite(leftWheelToggle, LOW);
  } else {
    digitalWrite(leftWheelToggle, HIGH);
  }
  analogWrite(rightMotorPWM, abs(rightSpeed));
  analogWrite(leftMotorPWM, abs(leftSpeed));
}

int getDistance()
{
  digitalWrite(distanceTrigger,HIGH);
  delayMicroseconds(10); //3 is the min for high
  digitalWrite(distanceTrigger,LOW);
  
  int time = pulseIn(distanceEcho,HIGH); //measures LOW to HIGH changes in pin 4
  float distance = (float) time / 58.77; //in cm
  
  float maxDistance = 100;
  int led = map(distance, 0, maxDistance, 0, 255);
  Serial.println(led);
  return led;
}
