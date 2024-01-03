/*
   rosserial::geometry_msgs::PoseArray Test
   Sums an array, publishes sum
*/
#include <Wire.h>
#include "ACS712.h"




//#include <ros.h>
//#include <std_msgs/String.h>
//#include <geometry_msgs/Pose.h>
//#include <geometry_msgs/PoseArray.h>

//ros::NodeHandle nh;
//bool set_;
//std_msgs::String str_msg;
//ros::Publisher p("encoder_feedback", &str_msg);
String encoder_msg = "";

#define pinEncRF2 4
#define pinEncRF1 5

#define pinEncLF1 6
#define pinEncLF2 7
#define pinEncLB1 8
#define pinEncLB2 9

#define pinEncRB2 10
#define pinEncRB1 11


#define pinBatteryVol A0
#define pinBatteryCur A1



#define R 248.18582;
#define pi 3.1416;

int i, j = 0, l, w = 0;
long pulseRF, pulseLF, pulseRB, pulseLB;
long prevPulseRF = 0, prevPulseLF = 0, prevPulseRB = 0, prevPulseLB = 0;
int speedRF, speedLF, speedRB, speedLB;
int prevSpeedRF = 0, prevSpeedLF = 0, prevSpeedRB = 0, prevSpeedLB = 0;
long prevTime = 0, curTime, delTime;
int ppr1 = 8544, ppr2 = 8576, ppr3 = 8544, ppr4 = 8576;
float mmpp1 = 1, mmpp2 = 1, mmpp3 = 1, mmpp4 = 1;

//void messageCb(const geometry_msgs::PoseArray& msg){
//  sum_msg.position.x = 0;
//  sum_msg.position.y = 0;
//  sum_msg.position.z = 0;
//  for(int i = 0; i < msg.poses_length; i++)
//  {
//    sum_msg.position.x += msg.poses[i].position.x;
//    sum_msg.position.y += msg.poses[i].position.y;
//    sum_msg.position.z += msg.poses[i].position.z;
//  }
//  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
//}

//ros::Subscriber<geometry_msgs::PoseArray> s("poses",messageCb);

ACS712  ACS(pinBatteryCur, 3.3, 1023, 520);

void pinModeDef() {
  for (i = 4; i < 12; i++) {
    pinMode(i, INPUT_PULLUP);
  }
}
//void pinModeDef1() {
//  for (i = 0; i < 4; i++) {
//    pinMode(pinIntV + i, OUTPUT);
//  }
//}



void setup()
{
//  mmpp1 = 248.18582 / 8544;
//  mmpp2 = 248.18582 / 8576;
//  mmpp3 = 248.18582 / 8544;
//  mmpp4 = 248.18582 / 8576;
  
  pinModeDef();
  pinMode(pinBatteryVol, INPUT);
  pinMode(pinBatteryCur, INPUT);
  //  pinModeDef1();
  Serial1.begin(57600);
  Serial.begin(115200);
  attachInterrupt(digitalPinToInterrupt(pinEncRF1), Change_RF, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinEncLF1), Change_LF, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinEncRB1), Change_RB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinEncLB1), Change_LB, CHANGE);


  ACS.autoMidPoint();
//  nh.initNode();
  //  nh.subscribe(s);
//  nh.advertise(p);
}

int delayTime = 25;
int voltage = 0, voltageAvg = 0, current = 0, currentAvg = 0, curMidValue = 508, curMultiplier = 100;
int cnt = 0, sample = 4;
int cnt_cur = 0, sample_cur = 40;
int cnt_vol = 0, sample_vol = 20;
void loop()
{
  curTime = millis();
  delTime = curTime - prevTime;
  speedLF = (pulseLF - prevPulseLF) * mmpp1 * 1000 / delTime;
  speedRF = (pulseRF - prevPulseRF) * mmpp2 * 1000 / delTime;
  speedLB = (pulseLB - prevPulseLB) * mmpp3 * 1000 / delTime;
  speedRB = (pulseRB - prevPulseRB) * mmpp4 * 1000 / delTime;

//  int mA = ACS.mA_DC();
  int mA = (analogRead(pinBatteryCur) - curMidValue) * 1000 * 3.33 / (curMultiplier * 5);
  
  if (cnt_cur < sample_cur){
    current = current + mA;
    cnt_cur++;
  }
  else{
    currentAvg = current / sample_cur;
    cnt_cur = 0;
    current = 0;
  }
  if (cnt_vol < sample_vol){
    voltage = voltage + analogRead(pinBatteryVol);
    cnt_vol++;
  }
  else{
    voltageAvg = voltage / sample_vol;
    cnt_vol = 0;
    voltage = 0;
  }

  prevPulseLF = pulseLF; prevPulseRF = pulseRF; prevPulseLB = pulseLB; prevPulseRB = pulseRB; 
  prevTime = curTime;
  
  encoder_msg = String(speedLF) + "," + String(speedRF) + "," + String(speedLB) + "," + String(speedRB)+ "," + String(voltageAvg) + "," + String(currentAvg);
  
//  int msg_len = encoder_msg.length() + 1;
//  char encoderMsgArray[msg_len];
//  encoder_msg.toCharArray(encoderMsgArray, msg_len);
  Serial1.println(encoder_msg);
  Serial.println(encoder_msg);

  delay(delayTime);
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