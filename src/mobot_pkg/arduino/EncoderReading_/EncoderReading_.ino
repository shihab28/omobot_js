#include <Wire.h>

#include "SerialTransfer.h"
SerialTransfer myTransfer;

#include <mbed.h>
#include <rtos.h>
#include <platform/Callback.h>
using namespace rtos;
Thread threadBLE;

#include <SimpleKalmanFilter.h>
 /*SimpleKalmanFilter(e_mea, e_est, q);
 e_mea: Measurement Uncertainty 
 e_est: Estimation Uncertainty 
 q: Process Noise
 */
SimpleKalmanFilter KalmanFilter1(.2, .1, 0.01);
SimpleKalmanFilter KalmanFilter2(.2, .1, 0.01);
SimpleKalmanFilter KalmanFilter3(.2, .1, 0.01);
SimpleKalmanFilter KalmanFilter4(.2, .1, 0.01);

#define pinIntV A0
#define pinVx A1
#define pinVy A2
#define pinWm A3

#define pinEncRF1 5
#define pinEncRF2 4

#define pinEncLF1 6
#define pinEncLF2 7

#define pinEncRB1 9
#define pinEncRB2 8

#define pinEncLB1 10
#define pinEncLB2 11

#define pinComMode 12

#define R 39.5
#define pi 3.1416
#define mmPerPulse .0291
#define pulsePerRotation 662

int charMode = 1;
// int ppr1 = 8560, ppr2 = 8576, ppr3 = 8560, ppr4 = 8576;
float mmpp1 = 0, mmpp2 = 0, mmpp3 = 0, mmpp4 = 0;
int ppr1 = 8544, ppr2 = 8576, ppr3 = 8544, ppr4 = 8576;

int currentSpeed = 20;
int i, j = 0, l, w = 0;
long pulseRF, pulseLF, pulseRB, pulseLB;
long prevPulseRF = 0, prevPulseLF = 0, prevPulseRB = 0, prevPulseLB = 0;
int speedRF, speedLF, speedRB, speedLB;
int prevSpeedRF = 0, prevSpeedLF = 0, prevSpeedRB = 0, prevSpeedLB = 0;

long prevTime = 0, curTime, delTime;


void pinModeDef() {
  for (i = 4; i < 12; i++) {
    pinMode(i, INPUT_PULLUP);
  }
}
void pinModeDef1() {
  for (i = 0; i < 4; i++) {
    pinMode(pinIntV + i, OUTPUT);
  }
}

void setup() {

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  Serial.begin(115200);
  Serial1.begin(115200);
  myTransfer.begin(Serial1);
  
  mmpp1 = R*2*pi/ppr1;
  mmpp2 = R*2*pi/ppr2;
  mmpp3 = R*2*pi/ppr3;
  mmpp4 = R*2*pi/ppr4;

  pinModeDef();
  pinModeDef1();
  analogWrite(pinIntV, 0);

  attachInterrupt(digitalPinToInterrupt(pinEncRF1), Change_RF, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinEncLF1), Change_LF, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinEncRB1), Change_RB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinEncLB1), Change_LB, CHANGE);

  analogWrite(pinIntV, 0);

  prevTime = millis();

  pinMode(pinComMode, OUTPUT);
  digitalWrite(pinComMode, LOW);
  delay(3000);
}

int delayTime = 25;

int encSpeedArray[20];

void loop() {
  // if (Serial1.available()) {
    curTime = millis();
    delTime = curTime - prevTime;
    speedLF = (pulseLF - prevPulseLF)*mmpp1*1000/delTime;
    speedRF = (pulseRF - prevPulseRF)*mmpp2*1000/delTime;
    speedLB = (pulseLB - prevPulseLB)*mmpp3*1000/delTime;
    speedRB = (pulseRB - prevPulseRB)*mmpp4*1000/delTime;

    // speedLF = (pulseLF - prevPulseLF)*mmpp1*1000/delTime + random(-4,4);
    // speedRF = (pulseRF - prevPulseRF)*mmpp2*1000/delTime + random(-4,4);
    // speedLB = (pulseLB - prevPulseLB)*mmpp3*1000/delTime + random(0,4);
    // speedRB = (pulseRB - prevPulseRB)*mmpp4*1000/delTime + random(0,4);

    // speedLF = KalmanFilter1.updateEstimate(speedLF);
    // speedRF = KalmanFilter1.updateEstimate(speedRF);
    // speedLB = KalmanFilter1.updateEstimate(speedLB);
    // speedRB = KalmanFilter1.updateEstimate(speedRB); 


    printEncoderValue();
    // printPulseValue();
    sendEncoderValue();
    
    prevPulseRF = pulseRF;
    prevPulseLF = pulseLF;
    prevPulseRB = pulseRB;
    prevPulseLB = pulseLB;
    prevTime = curTime;

    delay(delayTime-1);
  // }
}

void sendEncoderValue() {
  uint16_t sendSize = 0;
  sendSize = myTransfer.txObj(speedLF, sendSize); 
  sendSize = myTransfer.txObj(speedRF, sendSize);
  sendSize = myTransfer.txObj(speedLB, sendSize); 
  sendSize = myTransfer.txObj(speedRB, sendSize);
  myTransfer.sendData(sendSize);

}

void printEncoderValue() {
  Serial.print(speedLF); Serial.print(",");
  Serial.print(speedRF); Serial.print(",");
  Serial.print(speedLB); Serial.print(",");
  Serial.print(speedRB);
  Serial.print("\n");
}



void Change_RF() {
  if (digitalRead(pinEncRF2) == 0) {
    if (digitalRead(pinEncRF1) == 0) {
      pulseRF--;
    } else {
      pulseRF++;
    }
  } else {
    if (digitalRead(pinEncRF1) == 0) {
      pulseRF++;
    } else {
      pulseRF--;
    }
  }
}


void Change_LF() {
  if (digitalRead(pinEncLF2) == 0) {
    if (digitalRead(pinEncLF1) == 0) {
      pulseLF--;
    } else {
      pulseLF++;
    }
  } else {
    if (digitalRead(pinEncLF1) == 0) {
      pulseLF++;
    } else {
      pulseLF--;
    }
  }
}

void Change_RB() {
  if (digitalRead(pinEncRB2) == 0) {
    if (digitalRead(pinEncRB1) == 0) {
      pulseRB--;
    } else {
      pulseRB++;
    }
  } else {
    if (digitalRead(pinEncRB1) == 0) {
      pulseRB++;
    } else {
      pulseRB--;
    }
  }
}


void Change_LB() {
  if (digitalRead(pinEncLB2) == 0) {
    if (digitalRead(pinEncLB1) == 0) {
      pulseLB--;
    } else {
      pulseLB++;
    }
  } else {
    if (digitalRead(pinEncLB1) == 0) {
      pulseLB++;
    } else {
      pulseLB--;
    }
  }
}



void printPulseValue() {
  Serial.print(pulseLF);
  Serial.print("\t");
  Serial.print(pulseRF);
  Serial.print("\t");
  Serial.print(pulseLB);
  Serial.print("\t");
  Serial.print(pulseRB);
  Serial.print("\n");
}



