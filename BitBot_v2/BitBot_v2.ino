#include <Timers.h>
#include <Servo.h>

#define PUSHER_TIME 500
#define RIGHT 1
#define LEFT 0
#define RIGHT_MAX_SPEED 122
#define LEFT_MAX_SPEED 160
#define RIGHT_MAX_SPEED_BACK 123
#define SERVO_DELAY 500
#define STRAIGHTDELAYTIME 100
#define IDLE_TIME 3000
#define TWIRL_TIME 1000

/*---------------- Module Level Variables ---------------------------*/
//analog
#define serverSensorInput 11

#define frontTapeSensorInput 18
#define backTapeSensorInput 19

//digital
#define pusherEnable 6
#define pusherToggle 8

#define rightWheelToggle 2
#define leftWheelToggle 4
#define rightMotorPWM 3
#define leftMotorPWM 5

#define threeCoinDumpOut 9
#define fiveCoinDumpOut 10

#define frontBumperInput 15
#define backBumperInput 12

//states
int state;
#define COLLECT 0
#define BACKWARDS 1
#define LOOKING 4
#define ALIGNING 6
#define FORWARD 9
#define TO_SERVER 11
#define TO_EXCHANGE 13

//variables
int coinsGotten = 0;
int coinsWanted = 45;
int pushes = 0;
boolean hasDumped = false;
float TURN_SPEED = .5;
int turn_count = 0;
boolean HASNTBEENINSERVER = true;
int side = RIGHT;9

//Servos
Servo threeCoinDump;
Servo fiveCoinDump; 


void setup() { 
  Serial.begin(9600);
  Serial.println("Starting MEbot...");

  //pins
  pinMode(centerSensorInput, INPUT);
  pinMode(serverSensorInput, INPUT);
  pinMode(frontTapeSensorInput, INPUT);
  pinMode(backTapeSensorInput, INPUT);
 // pinMode(rightBumperInput, INPUT);
  pinMode(frontBumperInput, INPUT);
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
  threeCoinDump.write(170);
  fiveCoinDump.write(20);
  
  //pusher
  digitalWrite(pusherEnable, LOW);

  TMRArd_ClearTimerExpired(0);
  
  state = FORWARD;
  goForward();
  TMRArd_InitTimer(0, IDLE_TIME); 
  turn_count = 0;
}

void loop() {
  
  if (state == FORWARD) { 
   if (TestTimerExpired()) { 
      if (side == RIGHT) {
        lookAroundRight();
        turn_count += 1;
        if (turn_count > 3) {
          side = LEFT;
          turn_count = 0;
        }
      } else {
        lookAroundLeft();
        turn_count += 1;
        if (turn_count > 3) {
          side = RIGHT;
          turn_count = 0;
        }
      }
      TMRArd_InitTimer(0, TWIRL_TIME); 
      state = LOOKING;
    }

    if (frontAligned() || backAligned()) {
      lookAroundRight();
      state= ALIGNING;
    }
  }

  if (state == LOOKING) { 
    if (TestTimerExpired()) { 
      state = FORWARD;
      goForward();
      TMRArd_InitTimer(0, IDLE_TIME); 
    }

    if (frontAligned() || backAligned()) {
      lookAroundRight();
      state= ALIGNING;
    }
    
  }

    if (state == ALIGNING) { 
    if ((backAligned() && frontAligned() || serverLightSensed())) { 
      stopMotor();
      delay(50);
      lookAroundLeft();
      delay(800);
      stopMotor();
      delay(500);
      goForward();
      TMRArd_InitTimer(0, 5000);
     state = TO_SERVER;
    }
  }
 
  if (state == TO_SERVER) { 
    if (frontBumperHit()) { 
      stopMotor();
      getCoins();
      state = COLLECT;
    }
    if (TestTimerExpired()) { 
      goForward();
      state = FORWARD;
    }
  }
  
  if (state == COLLECT) {
    if(TestTimerExpired()) { 
      collect();
      goBackwards();
      delay(150);
      goForward();
      delay(200);
      stopMotor();
    }
    if (doneCollecting()) {
      goBackwards();
      TMRArd_InitTimer(0, 3700); 
      state = TO_EXCHANGE;
    }
  }
  
  if (state == TO_EXCHANGE) { 
    if (TestTimerExpired()) { 
       stopMotor();
       unloadFiveDumpServo();
       unloadThreeDumpServo();
       state = FORWARD;
    }
  }

}

void getCoins() { 
  hasDumped = false;
  coinsGotten = 0;
  pushes = 0;
  coinsWanted = 8;
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
  if (pushes == coinsGotten + 1) { 
    coinsGotten += 1;
    pushes = 0;
  }
}

void push() {
  digitalWrite(pusherEnable, HIGH);
  digitalWrite(pusherToggle, HIGH);
  delay(PUSHER_TIME);
  digitalWrite(pusherToggle, LOW);
  delay(100);
  digitalWrite(pusherEnable, LOW);
}

boolean doneCollecting() { 
  return coinsGotten == coinsWanted;
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

boolean frontBumperHit() {
  int currValue = digitalRead(frontBumperInput);
  return currValue == HIGH;
}

void goBackwards() {
  adjustMotorSpeed(-1 * (RIGHT_MAX_SPEED_BACK)/2, -1 * LEFT_MAX_SPEED/2);
}

void goForward() { 
  adjustMotorSpeed(RIGHT_MAX_SPEED/2, LEFT_MAX_SPEED/8*5);
  delay(STRAIGHTDELAYTIME);
  adjustMotorSpeed(RIGHT_MAX_SPEED/2,LEFT_MAX_SPEED/2);
}

void stopMotor() { 
  adjustMotorSpeed(0,0);
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

  for(int pos = 170; pos>=70; pos -= 1)     // goes from 180 degrees to 0 degrees 
  {                                
    threeCoinDump.write(pos);              // tell servo to go to position in variable 'pos' 
    delayMicroseconds(SERVO_DELAY);
  }
  delay(1000);
  for(int pos = 70; pos < 170; pos += 1)  // goes from 0 degrees to 180 degrees 
  {                                  // in steps of 1 degree 
    threeCoinDump.write(pos);              // tell servo to go to position in variable 'pos'
    delayMicroseconds(SERVO_DELAY);
  }    
}

void unloadFiveDumpServo() {
  for(int pos = 20; pos < 130; pos += 1)  // goes from 0 degrees to 180 degrees 
  {                                  // in steps of 1 degree 
    fiveCoinDump.write(pos);              // tell servo to go to position in variable 'pos'
    delayMicroseconds(DELAY);
  } 
  delay(1000);
  for(int pos = 130; pos >= 20; pos -= 1)     // goes from 180 degrees to 0 degrees 
  {                                
    fiveCoinDump.write(pos);              // tell servo to go to position in variable 'pos' 
    delayMicroseconds(DELAY);
  }  
}

void adjustMotorSpeed(int rightSpeed, int leftSpeed)
{
  if (rightSpeed < 0) {
    digitalWrite(rightWheelToggle, HIGH);
  } 
  else {
    digitalWrite(rightWheelToggle, LOW);
  }

  if (leftSpeed < 0) {
    digitalWrite(leftWheelToggle, LOW);
  } 
  else {
    digitalWrite(leftWheelToggle, HIGH);
  }
  analogWrite(rightMotorPWM, abs(rightSpeed));
  analogWrite(leftMotorPWM, abs(leftSpeed));
}

boolean frontAligned() { 
  boolean aligned = false;
  if (digitalRead(frontTapeSensorInput) == HIGH) aligned = true;
  return aligned;
}

boolean backAligned() { 
  boolean aligned = false;
  if (digitalRead(backTapeSensorInput) == HIGH) aligned = true;
  return aligned;
}
