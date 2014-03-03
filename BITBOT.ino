 #include <QTRSensors.h>
 #include <Servo.h>


/*---------------- Module Level Variables ---------------------------*/
//analog
int rightSensorInput = A0;
int leftSensorInput = A1;
int centerSensorInput = A2;
int serverSensorInput = A3;
int frontTapeSensorInput = A4;
int backTapeSensorInput = A5;

//digital
//int algorithmOut = 4;
int rightWheelToggle = 2;
int leftWheelToggle = 4;
int rightWheelPWM = 3;
int leftWheelPWM = 5;
int pusherToggle=8;
int threeCoinDumpOut = 9;
int fiveCoinDumpOut = 10;
int leftBumperInput=12;
int rightBumperInput=13;

byte byteRead;
int motorspeed=255;
//for collision logic 
int nextLeft;
int nextRight;
Servo myservo1;  // Creates a servo object
Servo myservo2;  // Creates a servo object
int pos = 0;    // Variable to store the servos angle 
int unloadpos=180;
int loadpos=90;

//states
int state;
int FIND_SERVER = 0; //going around looking for light
int GO_TO_SERVER = 1; //going straight until server is hit
int REVERSE = 2; //align 
int FORWARD = 4; //reach tape
int ALIGN_WITH_TAPE = 5; //turn forward
int GET_3_COINS = 6;
int GET_5_COINS = 7;
int TURNING = 8;
int TURN_AROUND = 9;
int GOING_TO_CENTER = 10;
int TURNING_TO_3 = 11;
int TURNING_TO_5 = 12;
int GOING_TO_3 = 13;
int GOING_TO_5 = 14;
int GOING_TO_8 = 15;
int GO_BACK = 16;
int DUMPING = 17;

//constants
int RIGHT = 1;
int LEFT = 0;
int MAX_TAPE_SENSOR_VAL = 1023;

//variables
int timeGoingBack = 2;
int timeGoingForward = 4;
int bumpedRightOrLeft; //right or left one is three
int coinsGotten = 0;
int coinsWanted = 0;
int pushes = 0;
int motorSpeed = 255;
int pusher_time_interval= 500; //half a second?
boolean hasDumped = false;
int linesSensed = 0;
int curr_tape_sensor_values[2];
char sequence_of_tape_sensor_changes[] = ""; //A front on, B front off, C back on, D back off

//sensors
QTRSensorsAnalog tapeSensors((unsigned char[]) {frontTapeSensorInput, backTapeSensorInput}, 2);


void setup() { 
  state = FIND_SERVER;
  myservo1.attach(threeCoinDumpOut);
  myservo2.attach(fiveCoinDumpOut);
  Serial.begin(9600);
  Serial.println("Starting MEbot...");
  //pins
  pinMode(rightSensorInput,INPUT);
  pinMode(leftSensorInput, INPUT);
  pinMode(centerSensorInput, INPUT);
  pinMode(serverSensorInput, INPUT);
  pinMode(frontTapeSensorInput, INPUT);
  pinMode(backTapeSensorInput, INPUT);
  pinMode(rightBumperInput, INPUT);
  pinMode(leftBumperInput, INPUT);
  pinMode(algorithmOut, OUTPUT);
  pinMode(rightWheelToggle, OUTPUT);
  pinMode(leftWheelToggle, OUTPUT);
  pinMode(rightWheelPWM, OUTPUT);
  pinMode(leftWheelPWM, OUTPUT);
  pinMode(threeCoinDumpOut, OUTPUT);
  pinMode(fiveCoinDumpOut, OUTPUT);
  pinMode(pusherToggle,OUTPUT);
  
//collision logic
  nextLeft=digitalRead(leftCollisionPin);
  nextRight=digitalRead(rightCollisionPin);
//testing
  digitalWrite(motor2toggle,HIGH);
  digitalWrite(motor1toggle,LOW);
  digitalWrite(pusherpin,HIGH);
  analogWrite(motor1pwm,255);
  analogWrite(motor2pwm,255);
//  Testparts
  push();
  unloadServo1();
//  myservo2.attach(servo2);
}

void loop() { 
	if (state == FIND_SERVER) { 
		if(serverLightSensed()) {
				serverFound();
				state = GO_TO_SERVER;
			} 
	}
	if (state == GO_TO_SERVER) { 
		if (rightBumperHit()) { 
			reverseFromRight();
			state = REVERSE;
		}
		if (leftBumperHit()) {
			reverseFromLeft();
			state = REVERSE;
		}
	}
	if (state == REVERSE) { 
		if (TestTimerExpired()) {
			goForward();
			state = FORWARD;
		}
	}
	if (state == FORWARD) { 
		if (TestTimerExpired()) {
			alignWithTape();
			state = ALIGN_WITH_TAPE;
		}
	}
	if (state == ALIGN_WITH_TAPE) { 
		if (alignedWithTape()) { 
			getThreeCoins();
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
			alignWithTape();
			state = TURN_AROUND;
		}

	}
	if (state == TURN_AROUND) { 
		if(alignedWithTape()) { 
			goForwardAlongTape();
			state = GOING_TO_CENTER;
		}
	}
	if (state == GOING_TO_CENTER) {
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
				goForward();
				state = GOING_TO_8;
			}
		}
	}
	if (state == TURNING_TO_3) { 
		if (TestTimerExpired()) {
			goForward();
			state = GOING_TO_3;
		}
	}
	if (state == TURNING_TO_5) {
		if (TestTimerExpired()) {
			goForward();
			state = GOING_TO_5;
		}
	}
	if (state == GOING_TO_5) {
		if (twoLinesSensed()) { 
			dumpFive();
			state = DUMPING;
		}
	}
	if (state == GOING_TO_3) {
		if (threeLinesSensed()) { 
			dumpThree();
			state = DUMPING;
		}
	}
	if (state == GOING_TO_8) {
		if (oneLineSensed()) {
			dumpEight();
			state = DUMPING;
		}
	}
	if (state == DUMPING) {
		hasDumped = true;
		if (doneDumping()) {
			goBack();
			state = GO_BACK;
		}
	}
	if (state == GO_BACK) {
		if (lineTapeIsSensed()) {
			alignWithTape();
			state = ALIGN_WITH_TAPE;
		}
	}

}

boolean serverLightSensed() { 
	return (digitalRead(serverSensorInput) == HIGH);
}

void serverFound() {
	//go straight
}

boolean rightBumperHit() { 
	int hit = digitalRead(rightBumperInput);
	if (hit == HIGH) return true;
	return false;
}

void reverseFromRight() {
	//set timer and reverse
	bumpedRightOrLeft = RIGHT;
}

boolean leftBumperHit() {
	int hit = digitalRead(leftBumperInput);
	if (hit == HIGH) return true;
	return false;
}

void reverseFromLeft() {
	bumpedRightOrLeft = LEFT;
}

void goForward() { 
	//set timer and go forward
}

void alignWithTape() {

}

boolean alignedWithTape() { 
	
}

void updateTapeSensorStatus() {

	int sensor_values[2];
	tapeSensors.read(sensor_values);
	if (sensor_values[0] > 3/4 * MAX_TAPE_SENSOR_VAL) { 
		curr_tape_sensor_values[0] = HIGH;
		sequence_of_tape_sensor_changes = sequence_of_tape_sensor_changes + 'A';
	}
	if (sensor_values[0] < 1/4 * MAX_TAPE_SENSOR_VAL) { 
		curr_tape_sensor_values[0] = LOW;
		sequence_of_tape_sensor_changes = sequence_of_tape_sensor_changes + 'B';
	}
	if (sensor_values[1] > 3/4 * MAX_TAPE_SENSOR_VAL) { 
		curr_tape_sensor_values[1] = HIGH;
		sequence_of_tape_sensor_changes = sequence_of_tape_sensor_changes + 'C';
	}
	if (sensor_values[1] < 1/4 * MAX_TAPE_SENSOR_VAL) { 
		curr_tape_sensor_values[1] = LOW;
		sequence_of_tape_sensor_changes = sequence_of_tape_sensor_changes + 'D';
	}

}

void getThreeCoins() { 
	if (hasDumped) coinsWanted = 11;
	else coinsWanted = 3;
	TMRArd_InitTimer(0, pusher_time_interval); 
}

void getFiveCoins() { 
	if (hasDumped) coinsWanted = 16;
	else coinsWanted = 8;
	TMRArd_InitTimer(0, pusher_time_interval); 
}


void pushAlgorithmButton() { 
	//push button
	pushes += 1;
	if (pushes == coinsGotten) { 
		coinsGotten +=1;
		pushed = 0;
	}

}


void collect() { 
	if (coinsGotten < coinsWanted) { 
		pushAlgorithmButton();
		TMRArd_InitTimer(0, pusher_time_interval);
	}
}

boolean doneCollecting() { 
	return coinsGotten == coinsWanted;
}

void turn() {
	//turn to collect five
}

void goForwardAlongTape() {

}

boolean tapeUnseen() {
	updateTapeSensorStatus();
	return curr_tape_sensor_values[0] == LOW and curr_tape_sensor_values[1] == LOW;
}


boolean threeIsAvailable() {
	int lightInput;
	if (bumpedRightOrLeft == RIGHT) { 
		lightInput = digitalRead(rightSensorInput);
	} else { 
		lightInput = digitalRead(leftSensorInput);
	}
	return (coinsGotten == 8 and lightInput == HIGH);
}

boolean fiveIsAvailable() {
	int lightInput;
	if (bumpedRightOrLeft == RIGHT) { 
		lightInput = digitalRead(leftSensorInput);
	} else { 
		lightInput = digitalRead(rightSensorInput);
	}
	return (coinsGotten == 8 and lightInput == HIGH);
}

boolean eightIsAvailable() {
	int lightInput = digitalRead(centerSensorInput);
	return (coinsGotten == 8 and lightInput == HIGH);
}

void turnTo3() {

}
void turnTo5() {
	//set timer
	//change motor directions
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

}

void doneDumping() { 

}

void goBack() { 

}

boolean lineTapeIsSensed() { 
	updateTapeSensorStatus();
	if (sequence_of_tape_sensor_changes == "CDAB") {
		sequence_of_tape_sensor_changes = "";
		return true;
	}
	return false;
}

unsigned char TestTimerExpired(void) {
  unsigned char value = (unsigned char)TMRArd_IsTimerExpired(0);
  if (value == TMRArd_EXPIRED) {
    TMRArd_ClearTimerExpired(0);
  }
  return value;
}
//This function sweeps the servo searching the area for an object within range
void unloadServo1()
{
  Serial.println("Unloading...");
  for(pos = unloadpos; pos>=loadpos; pos -= 1)     // goes from 180 degrees to 0 degrees 
  {                                
    myservo1.write(pos);              // tell servo to go to position in variable 'pos' 
    delayMicroseconds(DELAY);
  }
  for(pos = loadpos; pos < unloadpos; pos += 1)  // goes from 0 degrees to 180 degrees 
  {                                  // in steps of 1 degree 
    myservo1.write(pos);              // tell servo to go to position in variable 'pos'
    delayMicroseconds(DELAY);
  }    
}

void push()
{
  digitalWrite(pushertoggle,HIGH);
  delay(pusher_time_interval);
  digitalWrite(pushertoggle,LOW);
}
void toggleMotorDirection()
{
  PORTD= PORTD ^ B00010100;
}
void adjustMotorSpeed()
{
  analogWrite(motor1pwm,motorspeed);
  analogWrite(motor2pwm,motorspeed);
  Serial.println(motorspeed);

}
