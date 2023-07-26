#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16MultiArray.h>
ros::NodeHandle nh;
//std_msgs::Int16MultiArray wheel_pwm


#include "protothreads.h"
pt runMot1;
pt runMot2;
pt runMot3;
pt runMot4;
pt brakeMot;

#define pinMotRFpwm 2
#define pinMotRF2 4
#define pinMotRF1 3

#define pinMotLF1 6
#define pinMotLF2 5
#define pinMotLFpwm 7

#define pinMotRBpwm 11
#define pinMotRB1 A1
#define pinMotRB2 A0
//#define pinMotLBpwm 11
//#define pinMotLB2 A1
//#define pinMotLB1 A0
#define pinMotLBpwm 8
#define pinMotLB2 9
#define pinMotLB1 10


int W1, W2, W3, W4;
int i;
float Ln  = .1185, Wd  = .0825;
float SL = sqrt(Ln*Ln + Wd*Wd), r  = .0395, R  = 2*3.14159*r;
float LS = 1/SL;

int min_motor_speed = 30;
//void cmdVelCallback(const geometry_msgs::Twist& cmd_vel_msg) {

void cmdVelCallback(const std_msgs::Int16MultiArray& wheel_pwm_msg) {
//  float Vx = cmd_vel_msg.linear.x;
//  float Vy = cmd_vel_msg.linear.y;
//  float W0 = cmd_vel_msg.angular.z;

    W1 = wheel_pwm_msg.data[0];
    W2 = wheel_pwm_msg.data[1];
    W3 = wheel_pwm_msg.data[2];
    W4 = wheel_pwm_msg.data[3];

  checkMaxSpeed();
  if (abs(W1) >= min_motor_speed || abs(W2) >= min_motor_speed || abs(W3) >= min_motor_speed || abs(W4) >= min_motor_speed){
  runAllMotorFromSpeed();
  }
  else{
    brake();
  }
}
//ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", cmdVelCallback);
ros::Subscriber<std_msgs::Int16MultiArray> cmd_vel_sub("wheel_pwm", cmdVelCallback);


void initPinModeOutput(int pinFirst, int pinLast) {
  for (i = pinFirst; i <= pinLast; i++) {
    pinMode(i, OUTPUT);
  }
  pinMode(A0, OUTPUT);
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
}









void setup() {
  Serial.begin(115200);
  initPinModeOutput(pinMotLF1, 11);

  nh.initNode();
  nh.subscribe(cmd_vel_sub);
}

void loop() {
  nh.spinOnce();
}



















void checkMaxSpeed(){
  if (W1 > 255){W1 = 255;}else if (W1 < -255){W1 = -255;}
  if (W2 > 255){W2 = 255;}else if (W2 < -255){W2 = -255;}
  if (W3 > 255){W3 = 255;}else if (W3 < -255){W3 = -255;}
  if (W4 > 255){W4 = 255;}else if (W4 < -255){W4 = -255;}
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
  analogWrite(pinMotLBpwm, 255);
  digitalWrite(pinMotRB1, HIGH);
  digitalWrite(pinMotRB2, HIGH);
  analogWrite(pinMotRBpwm, 255);
  digitalWrite(pinMotLF1, HIGH);
  digitalWrite(pinMotLF2, HIGH);
  analogWrite(pinMotLFpwm, 255);
  digitalWrite(pinMotRF1, HIGH);
  digitalWrite(pinMotRF2, HIGH);
  analogWrite(pinMotRFpwm, 255);
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
