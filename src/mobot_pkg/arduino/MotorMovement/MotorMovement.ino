/*
    Omobot's Motor's Pulse Width Modulation Controller
    Author: Shihab Uddin Ahamad
    Email: shihab.ahamad28@gmail.com
    Date: 03/20/2024
    Description: 
    - This code is designed for controlling the motors of the Omobot using PWM control 
    - It receives the speed of each wheel as string data over serial communication from Jetson Nano and sends the 8-bit PWM pulses to L298N Motor driver

    Hardware Connections:
    - Motor controller connected to digital pins 2 through analog pin A1

    Note: This code is specifically tailored for Arduino boards and requires the ACS712 Arduino library.
*/


/*
    Omobot's Motor's Pulse Width Modulation Controller
    Author: Shihab Uddin Ahamad
    Email: shihab.ahamad28@gmail.com
    Date: 03/20/2024
    Description: 
    - This code is designed for controlling the motors of the Omobot using PWM control 
    - It receives the speed of each wheel as string data over serial communication from Jetson Nano and sends the 8-bit PWM pulses to L298N Motor driver

    Hardware Connections:
    - Motor controller connected to digital pins 2 through analog pin A1

    Note: This code is specifically tailored for Arduino boards and requires the ACS712 Arduino library.
*/

#include "protothreads.h"  // Include the ProtoThreads library for lightweight threading
#include "StringSplitter.h"  // Include the StringSplitter library for parsing received serial data

// Define protothread objects for running motor control and other functions
pt runMot1;
pt runMot2;
pt runMot3;
pt runMot4;
pt brakeMot;
pt lightBink;

// Define pin assignments for motor control
#define pinMotRFpwm 2
#define pinMotRF1 3
#define pinMotRF2 4
#define pinMotLF2 5
#define pinMotLFpwm 6
#define pinMotLF1 7
#define pinMotLB2 8
#define pinMotLBpwm 9
#define pinMotLB1 10
#define pinMotRBpwm 11
#define pinMotRB2 A0
#define pinMotRB1 A1

#define statusPin 13  // Pin for status indication (blinking LED)

// Variable declarations for wheel speed control and kinematics calculations
int W1, W2, W3, W4;
int min_motor_speed = 30;  // Minimum motor speed for operation
int runSequence[] = {0, 1, 2, 3};  // Motor run sequence for alternative coordinated movement

// Function to initialize all defined pins as output
void initPinModeOutput(int pinFirst, int pinLast) {
  for (i = pinFirst; i <= pinLast; i++) {
    pinMode(i, OUTPUT);
  }
  pinMode(A0, OUTPUT);
  pinMode(A1, OUTPUT);
  pinMode(statusPin, OUTPUT);
}

// Setup function for initial configurations
void setup() {
  Serial.begin(230400);       // Begin serial communication with high baud rate for receiving data from Jetson Nano
  initPinModeOutput(2, 11);  // Initialize motor control pins as output
}

// Main loop that continuously checks for new serial data and controls motors accordingly
void loop() {
  while (!Serial.available()) {
    // Wait for incoming serial data
  }
  
  if (Serial.available()) {
    serialData = Serial.readStringUntil('\n');  // Read the incoming data until newline character
    serialData.trim();  // Trim whitespace from the data
    
    PT_SCHEDULE(blinkLED(&lightBink));  // Blink the status LED to indicate data reception
    SplitString();  // Split the received string into individual speed values for each motor
    
    // Check if the received speeds are below a certain threshold and if so, brake
    if (abs(pwmSpeed[0]) < 50 && abs(pwmSpeed[1]) < 50 && abs(pwmSpeed[2]) < 50 && abs(pwmSpeed[3]) < 50) {
      brake();
    } else {
      // If speeds are above the threshold, control motors based on received speeds
      runAllMotorFromSpeed();
    }

    delay(refreshDelay);  // Short delay to allow for processing time and prevent flooding
  }
}

// Function to split the received string into individual motor speed values
void SplitString() {
  const int maxSplitParts = 4;  // Maximum number of parts to split the string into (corresponding to 4 motors)
  String splitParts[maxSplitParts];  // Array to hold the split string parts
  int splitCount = 0;  // Counter for the number of parts split
  int lastIndex = 0;  // Index of the last splitted part

  
  // Split the message with "," seperator, record the number of split and last index
   while (splitCount < maxSplitParts - 1) {
    int commaIndex = serialData.indexOf(',', lastIndex);
    
    if (commaIndex >= 0) { 
      splitParts[splitCount] = serialData.substring(lastIndex, commaIndex); 
      lastIndex = commaIndex + 1; 
      splitCount++; 
    } else {
      break;  // Exit loop if no more commas are found
    }
  }
  
  // Capture the last part of the message if it doesn't end with a comma
  if (lastIndex < serialData.length() && splitCount < maxSplitParts) {
    splitParts[splitCount] = serialData.substring(lastIndex);
    splitCount++; 
  }

  resetPwmSpeed();  // Reset speeds to 0 before setting new values
  // Convert split string parts to integers and assign them to pwmSpeed array
  for (int i = 0; i < splitCount; i++) {
    pwmSpeed[i] = splitParts[i].toInt();
  }
}


// Resets the PWM speed values for all motors to 0.
// This function is typically called before reading new speed values to ensure that outdated speeds are not used.
void resetPwmSpeed(){
   pwmSpeed[0] = 0; 
   pwmSpeed[1] = 0;
   pwmSpeed[2] = 0; 
   pwmSpeed[3] = 0;
}

// Blinks the status LED to indicate activity. This can be used as a visual
// signal for debugging or to show that the system is processing commands.
// The LED is turned on for 40 milliseconds and then off for 10 milliseconds.
int blinkLED(struct pt* pt) {
      digitalWrite(statusPin, HIGH); // Turn the LED on
      delay(40); // Wait for 40 milliseconds
      digitalWrite(statusPin, LOW); // Turn the LED off
      delay(10); // Wait for 10 milliseconds
}

// Ensures that the speed for each motor does not exceed the maximum allowable
// value of 255 or go below the minimum of -255. This function acts as a safety
// check to prevent sending invalid PWM values to the motor driver.
void checkMaxSpeed() {
  // Clamp the speed values for each motor to the range [-255, 255]
  for (int i = 0; i < 4; i++) {
    if (pwmSpeed[i] > 255) {
      pwmSpeed[i] = 255;
    } else if (pwmSpeed[i] < -255) {
      pwmSpeed[i] = -255;
    }
  }
}

// Runs all motors based on the current speed values stored in the pwmSpeed array.
// This function iterates through each motor, assigning the respective speed value
// and adjusting the motor control signals accordingly.
void runAllMotorFromSpeed() {
  // Temporary sequence array to ensure motors are run in order
  int tempSeq[4] = {0, 1, 2, 3};
  for (int j = 0; j < 4; j++) {
    runMotor(tempSeq[j], pwmSpeed[j]); // Run each motor with its designated speed
  }
}

signed int current_motorSpeed = 0; // Define the speed of the current motor to be controlled

// Controls the speed and direction of a specific motor based on the given speed.
// Positive values indicate one direction, while negative values indicate the reverse.
// This function maps the motor ID to the respective control pins and applies the PWM signal.
void runMotor(int motorId, signed int motorSpeed) {
  current_motorSpeed = motorSpeed; // Store current motor speed for use in thread functions
  // Schedule the appropriate thread function based on the motor ID
  switch(motorId) {
    case 0: PT_SCHEDULE(runMotorThread1(&runMot1)); break;
    case 1: PT_SCHEDULE(runMotorThread2(&runMot2)); break;
    case 2: PT_SCHEDULE(runMotorThread3(&runMot3)); break;
    case 3: PT_SCHEDULE(runMotorThread4(&runMot4)); break;
  }
}


// Thread functions for controlling each motor. These functions set the motor
// driver pins based on the direction indicated by the current speed (positive or negative).
// Each function handles a different motor, applying the PWM speed to the designated PWM control pin.
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


// Engages the braking mechanism for all motors by applying a high signal to both
// direction control pins of each motor driver. This function is useful for quickly
// stopping the robot or holding its position against external forces.
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

// Wrapper function to schedule the braking thread. This can be called to initiate
// braking on all motors.
void brake() {
  PT_SCHEDULE(brakeThread(&brakeMot));
}
