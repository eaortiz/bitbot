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
#define LEFT_MAX_SPEED 170
#define DELAY 500
#define TAPE_HIGH 10
#define TAPE_LOW 8
#define TIME_TO_MIDDLE 5000
#define TIME_TO_90 500
#define TIME_TO_180 1000

/*---------------- Module Level Variables ---------------------------*/
//analog
//#define rightSensorInput 17
//#define leftSensorInput 16
#define distanceTrigger 16
#define distanceEcho 17
#define centerSensorInput 18
#define serverSensorInput 11
#define frontTapeSensorInput 14
#define backTapeSensorInput 19

//digital
#define pusherEnable 1
#define rightWheelToggle 2
#define leftWheelToggle 4
#define rightMotorPWM 3
#define leftMotorPWM 5
#define backRightBumperInput 6
#define backLeftBumperInput 7
#define pusherToggle 8
#define threeCoinDumpOut 9
#define fiveCoinDumpOut 10
#define leftBumperInput 12
#define rightBumperInput 13

//states
int state;
#define GET_DIRECTION 0 //going around looking for light
#define GO_TO_MIDDLE 1
#define GO_TO_MIDDLE_2 3
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
#define REALIGN 18

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

//sensors
QTRSensorsAnalog tapeSensors((unsigned char[]) {frontTapeSensorInput, backTapeSensorInput}, 2);

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

  //calibrate tape sensors
  //for (int i = 0; i < 250; i++) // make the calibration take about 5 seconds
  //{
    //tapeSensors.calibrate();
    //delay(20);
  //}

  //wheels
 digitalWrite(rightWheelToggle, LOW);
 digitalWrite(leftWheelToggle, HIGH);
  
  //servos
  threeCoinDump.attach(threeCoinDumpOut);
  fiveCoinDump.attach(fiveCoinDumpOut);
  
  //collision logic
  nextLeft = digitalRead(leftBumperInput);
  nextRight = digitalRead(rightBumperInput);
  nextLeftBack = digitalRead(backLeftBumperInput);
  nextRightBack = digitalRead(backRightBumperInput);
  
  //pusher
  digitalWrite(pusherEnable, LOW);

  TMRArd_ClearTimerExpired(0);
// state = 1000;
  state = GET_DIRECTION;
  lookAroundRight();
  delay(10);
 // goForward();
}

int last_distance = 255;

void loop() { 
 // getDistance();
        if (state == GET_DIRECTION) { 
                if (getDistance() < 20) { 
                    stopMotor();
                    delay(1023);
                   goForward();
                   delay(1023);
                   lookAroundLeft();
                   state = GO_TO_MIDDLE;
                 // state = 1000;
                }
	}
        
        if (state == GO_TO_MIDDLE) { 
          Serial.println("mid");
          int curr_d = getDistance();
          if (last_distance > curr_d) { 
            last_distance = curr_d;
            //lookAroundLeft();
            stopMotor();
            delay(1023);
            goForward();
            delay(1023);
            if (last_distance == 255) { 
              state == 1000; //make it hte next one;
            } else { 
              lookAroundRight();
              state = GO_TO_MIDDLE_2;
             // state = 1000;
            }
          }
          
        if (state == GO_TO_MIDDLE_2) { 
          Serial.println("mid2");
            int curr_d = getDistance();
          if (last_distance > curr_d) { 
            last_distance = curr_d;
            stopMotor();
            delay(1023);
            goForward();
            delay(1023);
            if (last_distance == 255) { 
              state == 1000; //make it hte next one;
            } else { 
              lookAroundLeft();
              state = GO_TO_MIDDLE;
            }
          }
         // if(TestTimerExpired()) { 
           // stopMotor();
           // lookAround();
            //state = LOOKING;
          }

        }
        if (state == LOOKING) { 
          if (serverLightSensed()) { 
            goForward();
            state = APPROACH_SERVER;
          }
        }
        if (state == APPROACH_SERVER) { 
          if(rightBumperHit() || leftBumperHit()) { 
             stopMotor();
             state = GET_3_COINS;
          }
        }
	if (state == GET_3_COINS) { 
		if (TestTimerExpired()) { 
			collect();
		}
		if (doneCollecting()) { 
			turn();
			state = TURNING;
		}

	}
	if (state == TURNING) { 
		if (TestTimerExpired()) { 
			getFiveCoins();
			state = GET_5_COINS;
		}
	}
	if (state == GET_5_COINS) { 
		if (TestTimerExpired()) { 
			collect();
		}
		if (doneCollecting()) {
			bumpedRightOrLeft = RIGHT;
			turnBack();
			state = TURN_BACK;
		}

	}
	if (state == TURN_BACK) { 
		if(alignedWithTape()) { 
			goBackwards();
			state = GOING_TO_CENTER;
		}
	}
	if (state == GOING_TO_CENTER) {
		if(missaligned()) { 
			alignWithTape();
			state = REALIGN;
		}
		if(tapeUnseen()) { 
			if (threeIsAvailable()) { 
				turnTo3();
				state = TURNING_TO_3;
			}
			else if (fiveIsAvailable()) {
				turnTo5();
				state = TURNING_TO_5;
			}	
			else if (eightIsAvailable()) { 
				goBackwards();
				state = GOING_TO_8;
			}
			//else lookAround();
		}
	}
	if (state == REALIGN) { 
		if (alignedWithTape()) { 
			goBackwards();
			state = GOING_TO_CENTER;
		}
	}
	if (state == TURNING_TO_3) { 
		if (TestTimerExpired()) {
			goBackwards();
			state = GOING_TO_3;
		}
	}
	if (state == TURNING_TO_5) {
		if (TestTimerExpired()) {
			goBackwards();
			state = GOING_TO_5;
		}
	}
	if (state == GOING_TO_5) {
		if (atFive()) { 
			dumpFive();
			state = DUMPING;
		}
	}
	if (state == GOING_TO_3) {
		if (atThree()) { 
			dumpThree();
			state = DUMPING;
		}
	}
	if (state == GOING_TO_8) {
		if ((rightBackBumperHit() || leftBackBumperHit()) && oneLineSensed()) {
			dumpEight();
			state = DUMPING;
		}
	}
	if (state == DUMPING) {
		hasDumped = true;
		goForward();
		state = GO_BACK;
	}
	if (state == GO_BACK) {
		if (lineTapeIsSensed()) {
			alignWithTape();
			state = ALIGN_WITH_TAPE;
		}
	}

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
//  input = pulseIn(serverSensorInput, LOW, 1200) +  pulseIn(serverSensorInput, HIGH, 1200);
   input = pulseIn(serverSensorInput,LOW);
   inputtwo = pulseIn(serverSensorInput,HIGH);
   inputsum = input+inputtwo;
   Serial.println(input);
   Serial.println(inputtwo);
   Serial.println(inputsum);
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

void reverseFromRight() {
	TMRArd_InitTimer(0, TIME_BACKWARDS); 
	goBackwards();
	bumpedRightOrLeft = RIGHT;
	sideToAlign = RIGHT;
}

void reverseFromLeft() {
	TMRArd_InitTimer(0, TIME_BACKWARDS); 
	goBackwards();
	bumpedRightOrLeft = LEFT;
	sideToAlign = LEFT;
}

void goBackwards() { 
	adjustMotorSpeed(-1 * RIGHT_MAX_SPEED, -1 * LEFT_MAX_SPEED);
}

void goForward() { 
	adjustMotorSpeed(RIGHT_MAX_SPEED/2, LEFT_MAX_SPEED/2);
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

boolean alignedWithTape() {
	updateTapeSensorStatus();
        boolean res = (curr_tape_sensor_values[0] == HIGH and curr_tape_sensor_values[1] == HIGH);
       // Serial.println(res);
	return res;
}

void updateTapeSensorStatus() {

	unsigned int sensor_values[2];
	tapeSensors.read(sensor_values);
        Serial.println(sensor_values[0]);
        Serial.println(sensor_values[1]);
	if (sensor_values[0] > TAPE_HIGH) { 
               // Serial.println("front high");
		curr_tape_sensor_values[0] = HIGH;
		sequence_of_tape_sensor_changes += 'A';
	}
	if (sensor_values[0] < TAPE_LOW) { 
               // Serial.println("front low");
		curr_tape_sensor_values[0] = LOW;
		sequence_of_tape_sensor_changes = sequence_of_tape_sensor_changes + 'B';
	}
	if (sensor_values[1] > TAPE_HIGH) { 
                //Serial.println("back high");
		curr_tape_sensor_values[1] = HIGH;
		sequence_of_tape_sensor_changes = sequence_of_tape_sensor_changes + 'C';
	}
	if (sensor_values[1] < TAPE_LOW) { 
               // Serial.println("back low");
		curr_tape_sensor_values[1] = LOW;
		sequence_of_tape_sensor_changes = sequence_of_tape_sensor_changes + 'D';
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


void pushAlgorithmButton() { 
	push();
	pushes += 1;
	if (pushes == coinsGotten) { 
		coinsGotten +=1;
	}

}

void collect() { 
	if (coinsGotten < coinsWanted) { 
		pushAlgorithmButton();
		TMRArd_InitTimer(0, PUSHER_TIME);
	}
}

boolean doneCollecting() { 
	return coinsGotten == coinsWanted;
}

void turn() {
	adjustMotorSpeed(-1 * RIGHT_MAX_SPEED/2, -1 * LEFT_MAX_SPEED/2);
	delay(250);
	adjustMotorSpeed(RIGHT_MAX_SPEED/2, -1 * LEFT_MAX_SPEED);
	delay(125);
	adjustMotorSpeed(RIGHT_MAX_SPEED/2, LEFT_MAX_SPEED/2);
	delay(250);
	adjustMotorSpeed(-1 * RIGHT_MAX_SPEED/2, LEFT_MAX_SPEED/2);
	delay(125);
}

void turnBack() { 
	adjustMotorSpeed(-1 * RIGHT_MAX_SPEED/2, -1 * LEFT_MAX_SPEED/2);
	delay(250);
	adjustMotorSpeed(-1 * RIGHT_MAX_SPEED/2, LEFT_MAX_SPEED/2);
	delay(125);
	adjustMotorSpeed(RIGHT_MAX_SPEED/2, LEFT_MAX_SPEED/2);
	delay(250);
	adjustMotorSpeed(RIGHT_MAX_SPEED/2, -1 * LEFT_MAX_SPEED);
	delay(125);
	sideToAlign = LEFT;

}

boolean missaligned() { 
	updateTapeSensorStatus();
	if (curr_tape_sensor_values[0] == LOW or curr_tape_sensor_values[1] == LOW) { 
		if (sideToAlign == RIGHT) sideToAlign = LEFT;
		else sideToAlign = RIGHT;
		return true;	
	}
}

boolean tapeUnseen() {
	updateTapeSensorStatus();
	return curr_tape_sensor_values[0] == LOW and curr_tape_sensor_values[1] == LOW;
}

//boolean rightSensorOn() { 
//	int input = 2 * pulseIn(rightSensorInput, HIGH, 600);
//	return (input < 339 && input > 327);
//}

boolean centerSensorOn() { 
	int input = 2 * pulseIn(centerSensorInput, HIGH, 600);
	return (input < 339 && input > 327);
}

//boolean leftSensorOn() { 
//	int input = 2 * pulseIn(leftSensorInput, HIGH, 600);
//	return (input < 339 && input > 327);
//}

//TODO: fix to accomodate turning
boolean threeIsAvailable() {
	boolean beaconSensed;
	if (bumpedRightOrLeft == RIGHT) { 
	//	beaconSensed = rightSensorOn();
	} else { 
		//beaconSensed = leftSensorOn();
	}
	return (coinsGotten == 8 and beaconSensed);
}

boolean fiveIsAvailable() {
	int beaconSensed;
	if (bumpedRightOrLeft == RIGHT) { 
		//beaconSensed = leftSensorOn();
	} else { 
		//beaconSensed = rightSensorOn();
	}
	return (coinsGotten == 8 and beaconSensed);
}

boolean eightIsAvailable() {
	return (coinsGotten == 8 and centerSensorOn());
}

void turnTo3() {
	if (bumpedRightOrLeft == LEFT) {
		adjustMotorSpeed(-1 * RIGHT_MAX_SPEED/2, LEFT_MAX_SPEED/2);
	}
	if (bumpedRightOrLeft == RIGHT) { 
		adjustMotorSpeed(RIGHT_MAX_SPEED/2, -1 * LEFT_MAX_SPEED/2);
	}
	TMRArd_InitTimer(0, TURN_TIME);
}

void turnTo5() {
	if (bumpedRightOrLeft == RIGHT) {
		adjustMotorSpeed(-1 * RIGHT_MAX_SPEED/2, LEFT_MAX_SPEED/2);
	}
	if (bumpedRightOrLeft == LEFT) { 
		adjustMotorSpeed(RIGHT_MAX_SPEED/2, -1 * LEFT_MAX_SPEED/2);
	}
	TMRArd_InitTimer(0, TURN_TIME);
}

//A front on, B front off, C back on, D back off
boolean twoLinesSensed()  {
	updateTapeSensorStatus();
	if (sequence_of_tape_sensor_changes == "ABCDABCD") {
		sequence_of_tape_sensor_changes = "";
		return true;
	}
	return false;
}

void dumpFive() {
	unloadFiveDumpServo();
}

boolean threeLinesSensed() { 
	updateTapeSensorStatus();
	if (sequence_of_tape_sensor_changes == "ABCDABCDABCD") {
		sequence_of_tape_sensor_changes = "";
		return true;
	}
	return false;
}

void dumpThree() {
	unloadThreeDumpServo();
}

boolean oneLineSensed() { 
	updateTapeSensorStatus();
	if (sequence_of_tape_sensor_changes == "ABCD") {
		sequence_of_tape_sensor_changes = "";
		return true;
	}
	return false;
}

void dumpEight() { 
	unloadThreeDumpServo();
	unloadFiveDumpServo();
}

boolean lineTapeIsSensed() { 
	updateTapeSensorStatus();
	if (sequence_of_tape_sensor_changes == "CDAB") {
		sequence_of_tape_sensor_changes = "";
		return true;
	}
	return false;
}

boolean atFive() { 
	boolean bumped = false;
	if (bumpedRightOrLeft == LEFT) { 
		bumped = rightBackBumperHit();
	}
	if (bumpedRightOrLeft == RIGHT) {
		bumped = leftBackBumperHit();
	}
	twoLinesSensed();
}

boolean atThree() { 
	boolean bumped = false;
	if (bumpedRightOrLeft == RIGHT) { 
		bumped = rightBackBumperHit();
	}
	if (bumpedRightOrLeft == LEFT) {
		bumped = leftBackBumperHit();
	}
	threeLinesSensed();
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

void push() {
	digitalWrite(pusherEnable, HIGH);
  	digitalWrite(pusherToggle, HIGH);
  	delay(PUSHER_TIME);
  	digitalWrite(pusherToggle, LOW);
  	digitalWrite(pusherEnable, LOW);
}

void toggleMotorDirection()
{
  PORTD= PORTD ^ B00010100;
}

void adjustMotorSpeed(int rightSpeed, int leftSpeed)
{
//  Serial.println(rightSpeed);
//  Serial.println(leftSpeed);
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
  analogWrite(rightMotorPWM,abs(rightSpeed));
  analogWrite(leftMotorPWM,abs(leftSpeed));
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
 // last_distance = led;
  Serial.println(led);
  return led;
}
