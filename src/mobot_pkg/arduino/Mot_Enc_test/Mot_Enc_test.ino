

// #define motor_mode
#define pinMotRFpwm 2
#define pinMotRF1 3
#define pinMotRF2 4

#define pinMotLF2 5
#define pinMotLFpwm 6
#define pinMotLF1 7

#define pinMotLB2   8
#define pinMotLBpwm 9
#define pinMotLB1   10

#define pinMotRBpwm 11
#define pinMotRB2   A0
#define pinMotRB1   A1



#define pinEncRF2 4
#define pinEncRF1 5

#define pinEncLF1 6
#define pinEncLF2 7

#define pinEncLB1 8
#define pinEncLB2 9

#define pinEncRB2 10
#define pinEncRB1 11



#include "protothreads.h"
pt runMot1;
pt runMot2;
pt runMot3;
pt runMot4;
pt brakeMot;

int W1, W2, W3, W4, i, min_motor_speed = 30;
int serialData = 0;
String encoder_msg = "";
long pulseRF, pulseLF, pulseRB, pulseLB;
long prevPulseRF = 0, prevPulseLF = 0, prevPulseRB = 0, prevPulseLB = 0;
int speedRF, speedLF, speedRB, speedLB;
int prevSpeedRF = 0, prevSpeedLF = 0, prevSpeedRB = 0, prevSpeedLB = 0;
long prevTime = 0, curTime, delTime;
int ppr1 = 8544, ppr2 = 8576, ppr3 = 8544, ppr4 = 8576;
float mmpp1 = 1, mmpp2 = 1, mmpp3 = 1, mmpp4 = 1;

void initPinModeOutput(int pinFirst, int pinLast) {
    for (i = pinFirst; i <= pinLast; i++) {
      pinMode(i, OUTPUT);
    }
    pinMode(A0, OUTPUT);
    pinMode(A1, OUTPUT);
    pinMode(A2, OUTPUT);
  }

void pinModeDef() {
  for (i = 4; i < 12; i++) {
    pinMode(i, INPUT_PULLUP);
  }
}

#define enc_mode

void setup() {
  Serial.begin(115200);

  #ifdef motor_mode
  // initPinModeOutput(2, 11);
  #endif

  // #ifdef enc_mode
  pinModeDef();
  attachInterrupt(digitalPinToInterrupt(pinEncRF1), Change_RF, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinEncLF1), Change_LF, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinEncRB1), Change_RB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinEncLB1), Change_LB, CHANGE);
  // #endif

}


int delayTime = 100;
void loop() {

// #ifdef motor_mode
// W1 = -250; W2 = 250; W3 = -250; W4 = 250;

// Serial.print(W1); Serial.print(", "); Serial.print(W2); Serial.print(", "); Serial.print(W3); Serial.print(", "); Serial.println(W4);
// runAllMotorFromSpeed();
// delay(500);
// #endif


// #ifdef enc_mode
  
  curTime = millis();
  delTime = curTime - prevTime;
  speedLF = (pulseLF - prevPulseLF) * mmpp1 * 1000 / delTime;
  speedRF = (pulseRF - prevPulseRF) * mmpp2 * 1000 / delTime;
  speedLB = (pulseLB - prevPulseLB) * mmpp3 * 1000 / delTime;
  speedRB = (pulseRB - prevPulseRB) * mmpp4 * 1000 / delTime;

  prevPulseLF = pulseLF; prevPulseRF = pulseRF; prevPulseLB = pulseLB; prevPulseRB = pulseRB; 
  prevTime = curTime;

  encoder_msg = String(pulseLF) + "," + String(pulseRF) + "," + String(pulseLB) + "," + String(pulseRB);
  Serial.println(encoder_msg);

  delay(delayTime);
// #endif



}






void Change_RF() {
  if (digitalRead(pinEncRF2) == 0) {
    if (digitalRead(pinEncRF1) == 0) {
      pulseRF++;
    } else {
      pulseRF--;
    }
  } else {
    if (digitalRead(pinEncRF1) == 0) {
      pulseRF--;
    } else {
      pulseRF++;
    }
  }
}


void Change_LF() {
  if (digitalRead(pinEncLF2) == 0) {
    if (digitalRead(pinEncLF1) == 0) {
      pulseLF++;
    } else {
      pulseLF--;
    }
  } else {
    if (digitalRead(pinEncLF1) == 0) {
      pulseLF--;
    } else {
      pulseLF++;
    }
  }
}

void Change_RB() {
  if (digitalRead(pinEncRB2) == 0) {
    if (digitalRead(pinEncRB1) == 0) {
      pulseRB++;
    } else {
      pulseRB--;
    }
  } else {
    if (digitalRead(pinEncRB1) == 0) {
      pulseRB--;
    } else {
      pulseRB++;
    }
  }
}


void Change_LB() {
  if (digitalRead(pinEncLB2) == 0) {
    if (digitalRead(pinEncLB1) == 0) {
      pulseLB++;
    } else {
      pulseLB--;
    }
  } else {
    if (digitalRead(pinEncLB1) == 0) {
      pulseLB--;
    } else {
      pulseLB++;
    }
  }
}


void runAllMotorFromSpeed() {
  runMotor(0, W1);
  runMotor(1, W2);
  runMotor(2, W3);
  runMotor(3, W4);
}


signed int current_motorSpeed = 0;
void runMotor(int motorId, signed int motorSpeed) {
  current_motorSpeed = motorSpeed;
  if (motorId == 0) {
    // PT_SCHEDULE(runMotorThread1(&runMot1));
    runMotorThread1(&runMot1);
  } else if (motorId == 1) {
    // PT_SCHEDULE(runMotorThread2(&runMot2));
    runMotorThread2(&runMot2);
  } else if (motorId == 2) {
    // PT_SCHEDULE(runMotorThread3(&runMot3));
    runMotorThread3(&runMot3);
  } else if (motorId == 3) {
    // PT_SCHEDULE(runMotorThread4(&runMot4));
    runMotorThread4(&runMot4);
  }
}

int runMotorThread1(struct pt* pt) {
  if (current_motorSpeed >= 0) {
    digitalWrite(pinMotLF1, LOW);
    digitalWrite(pinMotLF2, HIGH);
    analogWrite(pinMotLFpwm, abs(current_motorSpeed));
  } else {
    digitalWrite(pinMotLF1, HIGH);
    digitalWrite(pinMotLF2, LOW);
    analogWrite(pinMotLFpwm, abs(current_motorSpeed));
  }
}

int runMotorThread2(struct pt* pt) {
  if (current_motorSpeed >= 0) {
    digitalWrite(pinMotRF1, HIGH);
    digitalWrite(pinMotRF2, LOW);
    analogWrite(pinMotRFpwm, abs(current_motorSpeed));
  } else {
    digitalWrite(pinMotRF1, LOW);
    digitalWrite(pinMotRF2, HIGH);
    analogWrite(pinMotRFpwm, abs(current_motorSpeed));
  }
}

int runMotorThread3(struct pt* pt) {
  if (current_motorSpeed >= 0) {
    digitalWrite(pinMotLB1, LOW);
    digitalWrite(pinMotLB2, HIGH);
    analogWrite(pinMotLBpwm, abs(current_motorSpeed));
  } else {
    digitalWrite(pinMotLB1, HIGH);
    digitalWrite(pinMotLB2, LOW);
    analogWrite(pinMotLBpwm, abs(current_motorSpeed));
  }
}

int runMotorThread4(struct pt* pt) {
  if (current_motorSpeed >= 0) {
    digitalWrite(pinMotRB1, LOW);
    digitalWrite(pinMotRB2, HIGH);
    analogWrite(pinMotRBpwm, abs(current_motorSpeed));
  } else {
    digitalWrite(pinMotRB1, HIGH);
    digitalWrite(pinMotRB2, LOW);
    analogWrite(pinMotRBpwm, abs(current_motorSpeed));
  }
}


int brakeThread(struct pt* pt) {
  digitalWrite(pinMotLB1, HIGH);
  digitalWrite(pinMotLB2, HIGH);
  analogWrite(pinMotLBpwm, 250);
  digitalWrite(pinMotRB1, HIGH);
  digitalWrite(pinMotRB2, HIGH);
  analogWrite(pinMotRBpwm, 250);
  digitalWrite(pinMotLF1, HIGH);
  digitalWrite(pinMotLF2, HIGH);
  analogWrite(pinMotLFpwm, 250);
  digitalWrite(pinMotRF1, HIGH);
  digitalWrite(pinMotRF2, HIGH);
  analogWrite(pinMotRFpwm, 250);
}

int TargEncSpeed[4] = {0, 0, 0, 0};
void brake() {
  // Serial.println("Braking");
//  TargEncSpeed[0] = 0;
//  TargEncSpeed[1] = 0;
//  TargEncSpeed[2] = 0;
//  TargEncSpeed[3] = 0;
  PT_SCHEDULE(brakeThread(&brakeMot));
}
