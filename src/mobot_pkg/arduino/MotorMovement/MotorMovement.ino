//#include <ros.h>
//#include <geometry_msgs/Twist.h>
//#include <std_msgs/Int16MultiArray.h>
//ros::NodeHandle nh;
//std_msgs::Int16MultiArray wheel_pwm


#include "protothreads.h"
#include "StringSplitter.h"
//#include <StringAction.h>
//StringAction pwmReader;

pt runMot1;
pt runMot2;
pt runMot3;
pt runMot4;
pt brakeMot;
pt lightBink;

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

#define statusPin 13

int W1, W2, W3, W4;
float Ln  = .1185, Wd  = .0825;
float SL = sqrt(Ln * Ln + Wd*Wd), r  = .0398, R  = 2 * 3.14159 * r;
float LS = 1 / SL;
int i, j, k, l, n;
int min_motor_speed = 30;
int runSequence = {0, 1, 2, 3};
//void cmdVelCallback(const geometry_msgs::Twist& cmd_vel_msg) {

//void cmdVelCallback(const std_msgs::Int16MultiArray& wheel_pwm_msg) {
////  float Vx = cmd_vel_msg.linear.x;
////  float Vy = cmd_vel_msg.linear.y;
////  float W0 = cmd_vel_msg.angular.z;
//
//    W1 = wheel_pwm_msg.data[0];
//    W2 = wheel_pwm_msg.data[1];
//    W3 = wheel_pwm_msg.data[2];
//    W4 = wheel_pwm_msg.data[3];
//
//  checkMaxSpeed();
//  if (abs(W1) >= min_motor_speed || abs(W2) >= min_motor_speed || abs(W3) >= min_motor_speed || abs(W4) >= min_motor_speed){
//  runAllMotorFromSpeed();
//  }
//  else{
//    brake();
//  }
//}
//ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", cmdVelCallback);
//ros::Subscriber<std_msgs::Int16MultiArray> cmd_vel_sub("wheel_pwm", cmdVelCallback);


void initPinModeOutput(int pinFirst, int pinLast) {
  for (i = pinFirst; i <= pinLast; i++) {
    pinMode(i, OUTPUT);
  }
  pinMode(A0, OUTPUT);
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(statusPin, OUTPUT);
}









void setup() {
  Serial.begin(230400);
  //  Serial1.begin(57600);
  initPinModeOutput(2, 11);

  //  nh.initNode();
  //  nh.subscribe(cmd_vel_sub);
}



int refreshDelay = 10;
String splitedString[4] = {"0", "0", "0", "0"};
int pwmSpeed[4] = {0, 0, 0, 0};
String serialData;
String tempStr;
int itemCount;
void loop() {
  //  nh.spinOnce();

  while (!Serial.available()) {
//    delay(10);
  }
  
  if (Serial.available()) {
    serialData  = Serial.readStringUntil('\n');
    serialData.trim();
//    Serial1.println(serialData);
      
    PT_SCHEDULE(blinkLED(&lightBink));
    SplitString();

    
//    StringSplitter *splitter = new StringSplitter(serialData, ',', 4);
//    itemCount = splitter->getItemCount();
//    if (itemCount == 4) {
//      for (i = 0; i < itemCount; i++) {
//        tempStr = splitter->getItemAtIndex(i);
//        pwmSpeed[i] = tempStr.toFloat();
//      }
//      
//    }
//
//    else {
//      pwmSpeed[0] = 0; pwmSpeed[1] = 0; pwmSpeed[2] = 0; pwmSpeed[3] = 0;
//      brake();
//    }

    if (abs(pwmSpeed[0]) < 50 && abs(pwmSpeed[1]) < 50 && abs(pwmSpeed[2]) < 50 && abs(pwmSpeed[3]) < 50){
      brake();
    }
    else{
      runAllMotorFromSpeed();
    }
    
    delay(refreshDelay);
  }



}


void SplitString(){
  const int maxSplitParts = 4; 
  String splitParts[maxSplitParts]; 
  int splitCount = 0; 
  int lastIndex = 0; 
  
  while (splitCount < maxSplitParts - 1) {
    int commaIndex = serialData.indexOf(',', lastIndex);
    
    if (commaIndex >= 0) { 
      splitParts[splitCount] = serialData.substring(lastIndex, commaIndex); 
      lastIndex = commaIndex + 1; 
      splitCount++; 
    } else {
      break; 
    }
  }
  
  if (lastIndex < serialData.length() && splitCount < maxSplitParts) {
    splitParts[splitCount] = serialData.substring(lastIndex);
    splitCount++; 
  }

  resetPwmSpeed();
  for (int i = 0; i < splitCount; i++) {
    pwmSpeed[i] = splitParts[i].toInt();
  }
}


void resetPwmSpeed(){
   pwmSpeed[0] = 0; 
   pwmSpeed[1] = 0;
   pwmSpeed[2] = 0; 
   pwmSpeed[3] = 0;
}

int blinkLED(struct pt* pt) {
      digitalWrite(statusPin, HIGH);
      delay(40);
      digitalWrite(statusPin, LOW);
      delay(10);
//      digitalWrite(statusPin, HIGH);
//      delay(30);
//      digitalWrite(statusPin, LOW);
//      delay(10);
}













void checkMaxSpeed() {
  if (pwmSpeed[0] > 255) {
    pwmSpeed[0] = 255;
  } else if (pwmSpeed[0] < -255) {
    pwmSpeed[0] = -255;
  }
  if (pwmSpeed[1] > 255) {
    pwmSpeed[1] = 255;
  } else if (pwmSpeed[1] < -255) {
    pwmSpeed[1] = -255;
  }
  if (pwmSpeed[2] > 255) {
    pwmSpeed[2] = 255;
  } else if (pwmSpeed[2] < -255) {
    pwmSpeed[2] = -255;
  }
  if (pwmSpeed[3] > 255) {
    pwmSpeed[3] = 255;
  } else if (pwmSpeed[3] < -255) {
    pwmSpeed[3] = -255;
  }
}

void runAllMotorFromSpeed() {
  int tempSeq
  tempSeq = runSequence;
  for (int j = 0; j < 4; j++) {
    runMotor(runSequence[j], pwmSpeed[j]);
    tempSeq[j] = (j + 1) % 4;
    //    Serial.print(pwmSpeed[j]); Serial.print(" | ");
    //    Serial1.print(pwmSpeed[j]); Serial1.print(" | ");
  }
  runSequence = tempSeq;

  //  Serial.println("");

  //  runMotor(1, pwmSpeed[1]);
  //  runMotor(2, pwmSpeed[2]);
  //  runMotor(3, pwmSpeed[3]);
}


signed int current_motorSpeed = 0;

void runMotor(int motorId, signed int motorSpeed) {
  current_motorSpeed = motorSpeed;
  if (motorId == 0) {
    PT_SCHEDULE(runMotorThread1(&runMot1));
    //    runMotorThread1(&runMot1);
  } else if (motorId == 1) {
    PT_SCHEDULE(runMotorThread2(&runMot2));
    //    runMotorThread2(&runMot2);
  } else if (motorId == 2) {
    PT_SCHEDULE(runMotorThread3(&runMot3));
    //    runMotorThread3(&runMot3);
  } else if (motorId == 3) {
    PT_SCHEDULE(runMotorThread4(&runMot4));
    //    runMotorThread4(&runMot4);
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
