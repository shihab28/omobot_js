/*
    Omobot's Wheel Speed and Power Monitoring
    Author: Shihab Uddin Ahamad
    Email: shihab.ahamad28@gmail.com
    Date: 03/20/2024
    Description: 
    - This code is designed for Omobot with encoders on each wheel for getting rotational speed of the wheel, and an ACS712 sensor for power monitoring. 
    - It calculates and send the speed of each wheel, battery voltage and current as string data over serial communication to Jetson Nano.

    Hardware Connections:
    - Encoders connected to digital pins 4 through 11
    - ACS712 sensor for current measurement connected to A1
    - Battery voltage measurement connected to A0

    Note: This code is specifically tailored for Arduino boards and requires the ACS712 Arduino library.
*/

#include "ACS712.h"  // Include library for ACS712 current sensor

String encoder_msg = "";  // String to store encoder messages

// Define digital pins for encoder connections
#define pinEncRF2 4
#define pinEncRF1 5
#define pinEncLF1 6
#define pinEncLF2 7
#define pinEncLB1 8
#define pinEncLB2 9
#define pinEncRB2 10
#define pinEncRB1 11

// Define analog pins for battery voltage and current measurements
#define pinBatteryVol A0
#define pinBatteryCur A1

// Define constants for calculations
#define R 248.18582;
#define pi 3.1416;

// Variable declarations for encoder and speed calculations
int i, j = 0, l, w = 0;
long pulseRF, pulseLF, pulseRB, pulseLB;
long prevPulseRF = 0, prevPulseLF = 0, prevPulseRB = 0, prevPulseLB = 0;
int speedRF, speedLF, speedRB, speedLB;
int prevSpeedRF = 0, prevSpeedLF = 0, prevSpeedRB = 0, prevSpeedLB = 0;
long prevTime = 0, curTime, delTime;
int ppr1 = 8544, ppr2 = 8576, ppr3 = 8544, ppr4 = 8576;  // Pulses per revolution for each encoder
float mmpp1 = 1, mmpp2 = 1, mmpp3 = 1, mmpp4 = 1;  // Millimeters per pulse

// Initialize ACS712 current sensor
ACS712  ACS(pinBatteryCur, 3.3, 1023, 520);

// Function to set pin modes for encoder inputs
void pinModeDef() {
  for (i = 4; i < 12; i++) {
    pinMode(i, INPUT_PULLUP);  // Set encoder pins as input with pull-up
  }
}

// Setup function runs once at the start
void setup() {
  pinModeDef();  // Set pin modes for encoders
  pinMode(pinBatteryVol, INPUT);  // Set pin mode for battery voltage
  pinMode(pinBatteryCur, INPUT);  // Set pin mode for battery current
  
  Serial1.begin(57600);  // Begin Serial1 communication
  Serial.begin(115200);  // Begin Serial  communication for debugging
  
  // Attach interrupt service routines for encoder signal changes
  attachInterrupt(digitalPinToInterrupt(pinEncRF1), Change_RF, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinEncLF1), Change_LF, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinEncRB1), Change_RB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinEncLB1), Change_LB, CHANGE);

  ACS.autoMidPoint();  // Calibrate the zero current point
}

// Main loop function runs repeatedly
void loop() {
  curTime = millis();  // Get current time
  delTime = curTime - prevTime;  // Calculate time difference
  
  // Calculate speeds for each wheel
  speedLF = (pulseLF - prevPulseLF) * mmpp1 * 1000 / delTime;
  speedRF = (pulseRF - prevPulseRF) * mmpp2 * 1000 / delTime;
  speedLB = (pulseLB - prevPulseLB) * mmpp3 * 1000 / delTime;
  speedRB = (pulseRB - prevPulseRB) * mmpp4 * 1000 / delTime;

  // Measure battery current
  int mA = (analogRead(pinBatteryCur) - curMidValue) * 1000 * 3.33 / (curMultiplier * 5);
  
  // Average current over several samples for stability
  if (cnt_cur < sample_cur){
    current += mA;
    cnt_cur++;
  }
  else{
    currentAvg = current / sample_cur;
    cnt_cur = 0;
    current = 0;
  }
  
  // Average voltage over several samples for stability
  if (cnt_vol < sample_vol){
    voltage += analogRead(pinBatteryVol);
    cnt_vol++;
  }
  else{
    voltageAvg = voltage / sample_vol;
    cnt_vol = 0;
    voltage = 0;
  }

  // Update previous pulse counts and time
  prevPulseLF = pulseLF; prevPulseRF = pulseRF; prevPulseLB = pulseLB; prevPulse
  
  prevTime = curTime; // Set current time as past time for next loop
  
  // Create String with the speed of the four wheels, current and voltage readings for sending over Serial1
  encoder_msg = String(speedLF) + "," + String(speedRF) + "," + String(speedLB) + "," + String(speedRB)+ "," + String(voltageAvg) + "," + String(currentAvg);
  
  // int msg_len = encoder_msg.length() + 1;
  // char encoderMsgArray[msg_len];
  // encoder_msg.toCharArray(encoderMsgArray, msg_len);

  // Send the String message over Serial1 and print in Serial port for debugginh
  Serial1.println(encoder_msg);
  Serial.println(encoder_msg);


  delay(delayTime);
}



// Update the pulses interrrupted the the encoder of the right-forward wheel
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

// Update the pulses interrrupted the the encoder of the left-forward wheel
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

// Update the pulses interrrupted the the encoder of the right-back wheel
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

// Update the pulses interrrupted the the encoder of the left-back wheel
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



