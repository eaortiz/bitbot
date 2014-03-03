 #include <QTRSensors.h>

int rightSensorInput = A0;
int leftSensorInput = A1;
int centerSensorInput = A2;
int serverSensorInput = A3;
int frontTapeSensorInput = A4;
int backTapeSensorInput = A5;
int rightBumperInput = A6;
int leftBumperInput = A7;

int algorithmOut = 4;
int rightWheelToggle = 3;
int leftWheelToggle = 5;
int rightWheelPWM = 6;
int leftWheelPWM = 7;
int threeCoinDumpOut = 8;
int fiveCoinDumpOut = 9;

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

//variables
int timeGoingBack = 2;
int timeGoingForward = 4;
int bumpedRightOrLeft; //right or left one is three
int coinsGotten = 0;
int coinsWanted = 0;
int pushes = 0;
int motorSpeed = 255;

//sensors
QTRSensorsAnalog tapeSensors((unsigned char[]) {frontTapeSensorInput, backTapeSensorInput}, 2);


void setup() { 
  state = FIND_SERVER;

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

void serverLightSensed() { 
}

void serverFound() {
	//stop and go straight
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

}

void goForward() { 
	//set timer and go forward
}

void alignWithTape() {

}

boolean alignedWithTape() { 
	//test tape sensors
}

void getThreeCoins() { 
	//set coinsWanted
	//set timer for first coin

}



void getFiveCoins() { 
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
		TMRArd_InitTimer(0, TIME_INTERVAL); //set timer
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

boolean twoLinesSensed()  {

}

void dumpFive() {

}

boolean threeLinesSensed() { 

}

void dumpThree() {

}

void oneLineSensed() { 

}

void dumpEight() { 

}

void doneDumping() { 

}

void goBack() { 

}

boolean lineTapeIsSensed() { 

}

unsigned char TestTimerExpired(void) {
  unsigned char value = (unsigned char)TMRArd_IsTimerExpired(0);
  if (value == TMRArd_EXPIRED) {
    TMRArd_ClearTimerExpired(0);
  }
  return value;
}