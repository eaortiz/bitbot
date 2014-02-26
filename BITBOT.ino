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
int rightWheelOut = 3;
int leftWheelOut = 5;
int threeCoinDumpOut = 6;
int fiveCoinDumpOut = 7;

//states
int state;
int FIND_SERVER = 0; //going around looking for light
int GO_TO_SERVER = 1; //going straight until server is hit
int REVERSE = 2; //align 
int GO_FORWARD = 4;
int ALIGN_WITH_TAPE = 5;
int GET_COINS = 6;

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

//sensors
QTRSensorsAnalog tapeSensors((unsigned char[]) {frontTapeSensorInput, backTapeSensorInput}, 2);


void setup() { 
  state = FIND_SERVER;
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
		if (TestTimerExpired) {
			goForward();
			state = FORWARD;
		}
	}
	if (state == FORWARD) { 
		if (TestTimerExpired) {
			alignWithTape();
			state = ALIGN_WITH_TAPE;
		}
	}
	if (state = ALIGN_WITH_TAPE) { 
		if (alignedWithTape()) { 
			getThreeCoins();
			state = GET_3_COINS;
		}
	}
	if (state = GET_3_COINS) { 
		if (TestTimerExpired) { 
			collect();
		}
		if (doneCollecting()) { 
			turn();
			state = TURNING;
		}

	}
	if (state = TURNING) { 
		if (TestTimerExpired) { 
			getFiveCoins();
			state = GET_5_COINS;
		}
	}
	if (state = GET_5_COINS) { 
		if (TestTimerExpired) { 
			collect();
		}
		if (doneCollecting) {
			bumpedRightOrLeft = RIGHT;
			alignWithTape();
			state = TURN_AROUND;
		}

	}
	if (state = TURN_AROUND) { 
		if(alignedWithTape()) { 
			goForwardAlongTape();
			state = GOING_TO_CENTER;
		}
	}
	if (state = GOING_TO_CENTER) {
		if(tapeUnseen()) { 
			if (threeIsAvailable()) { 
				turnTo3();
				state = TURNING_TO_3;
			}
			else if (fiveIsAvailable()) {
				turnTo5();
				state = TURNING_TO_5;
			}	
		}
	}
	if (state = TURNING_TO_3) { 
		if (TestTimerExpired) {
			goForward();
			state = GOING_TO_3;
		}
	}
	if (state = TURNING_TO_5) {
		if (TestTimerExpired) {
			goForward();
			state = GOING_TO_5;
		}
	}
	if (state = GOING_TO_5) {
		if(2linesSensed()) { 
			dumpFive();
			state = DUMPING_5;
		}
	}
	if (state = GOING_TO_3) {

	}
	if (state = GOING_TO_8) {

	}
	if (state = )




}

void serverLightSensed() { 
}

void serverFound(){
	//stop and go straight
}

void reverseFromRight() {
	//set timer and reverse
	bumpedRightOrLeft = RIGHT;
}

void goForward() { 
	//set timer and go forward
}

void getThreeCoins() { 
	//set coinsWanted
	//set timer for first coin

}

void getFiveCoins() { 
}

void alignedWithTape() { 
	//test tape sensors
}

void pushAlgorithmButton() { 
	//push button
	pushes += 1;
	if (pushes == coinsGotten) { 
		coinsGotten +=1;
		pushed = 0;
	}

}

void moveDiagonally(){
	move();
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
void getToServer() { 
	//turn around
	//when light is found
	state = SERVER_FOUND;
}

void goBackwards() { 
	TMRArd_InitTimer(0, TIME_INTERVAL);
}

void pushAlgorithmButton() { 
}

void turnTo5() {
	//set timer
	//change motor directions
}
unsigned char TestTimerExpired(void) {
  unsigned char value = (unsigned char)TMRArd_IsTimerExpired(0);
  if (value == TMRArd_EXPIRED) {
    TMRArd_ClearTimerExpired(0);
  }
  return value;
}