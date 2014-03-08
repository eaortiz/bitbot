#define frontReceiver 18
#define backReceiver 19

#define rightWheelToggle 2
#define leftWheelToggle 4
#define rightMotorPWM 3
#define leftMotorPWM 5

#define RIGHT 0
#define LEFT 1
#define START 2
#define STRAIGHT 0
#define ALIGNING 1


#define RIGHT_MAX_SPEED 125
#define LEFT_MAX_SPEED 170

float TURN_SPEED = .5;

int side_to_turn = RIGHT;
int state;

void setup() { 
  state = START;
  pinMode(frontReceiver, INPUT);
  pinMode(backReceiver, INPUT);
  pinMode(rightWheelToggle, OUTPUT);
  pinMode(leftWheelToggle, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);
  Serial.begin(9600);
  Serial.println("start");
  goForward();
  delay(50);
  
}

void loop() { 
  if (state == START) { 
    if (!isAligned()) { 
      Serial.println("sup");
      stopMotor();
//      goForward();
  //    delay(500);
    //  if (side_to_turn == RIGHT) { 
      //  lookAroundRight();
//        side_to_turn = LEFT;
  ///    } else { 
     //   lookAroundLeft();
       // side_to_turn = RIGHT;
      //}
      //state = ALIGNING;
   
    }
  }
  //when in this state
  if (state == STRAIGHT) {
    if (!isAligned()) { 
      
      if (side_to_turn == RIGHT) { 
        lookAroundRight();
        side_to_turn = LEFT;
      } else { 
        lookAroundLeft();
        side_to_turn = RIGHT;
      }
      state == ALIGNING;
    } 
  }
  
  if (state == ALIGNING) { 
    if (isAligned()) { 
      stopMotor();
      delay(500);
      goForward();
      state = STRAIGHT;
      
    }
  }
} 

void lookAroundRight() { 
        adjustMotorSpeed(-1 * (float) RIGHT_MAX_SPEED * TURN_SPEED, (float) LEFT_MAX_SPEED * TURN_SPEED);
}

void lookAroundLeft() { 
        adjustMotorSpeed((float) RIGHT_MAX_SPEED * TURN_SPEED, -1 * (float) LEFT_MAX_SPEED * TURN_SPEED);
}

boolean frontAligned() { 
  boolean aligned = false;
  if (digitalRead(frontReceiver) == HIGH) aligned = true;
  return aligned;
  
}
boolean isAligned() { 
  boolean aligned = frontAligned();
   if (digitalRead(backReceiver) == HIGH) aligned = aligned && true;  
  return aligned;
} 

void goForward() { 
  Serial.println("go");
	adjustMotorSpeed(RIGHT_MAX_SPEED/2, LEFT_MAX_SPEED/2);
}

void stopMotor() { 
  adjustMotorSpeed(0,0);
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
