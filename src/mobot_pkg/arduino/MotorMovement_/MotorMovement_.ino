// #define kp .06
// #define kd .2
// #define ki 0

// #define SERIAL_RX_BUFFER_SIZE 1024
// #define SERIAL_TX_BUFFER_SIZE 512



#include <serial-readline.h>
void SerialDataReceived(char*);
SerialLineReader reader(Serial);

float kp[4] = { .082, .095, .080, .080 };
float kd[4] = { .020, .020, .020, .020 };
float ki[4] = { .002, .002, .002, .002 };
int delayTime = 50;

// #define SERIAL_CDC

#include <Wire.h>

#include <mbed.h>
#include <rtos.h>
#include <platform/Callback.h>
using namespace rtos;
Thread threadBLE;
Thread threadEnc;
long curTime;

#include "SerialTransfer.h"
SerialTransfer myTransfer;

#include "protothreads.h"
pt runMot1;
pt runMot2;
pt runMot3;
pt runMot4;
pt brakeMot;

#include "StringSplitter.h"
String serialData, serialSplitData[4];
String serialEncoderData, serialEncoderSplitData[4];

#include <ArduinoBLE.h>
const char* serviceUUID = "00000000-0000-0000-0000-000000111111";
const char* CHLH_UUID = "00000000-0000-0000-0000-000000000001";
const char* CHLV_UUID = "00000000-0000-0000-0000-000000000010";
const char* CHRH_UUID = "00000000-0000-0000-0000-000000000100";
const char* CHRV_UUID = "00000000-0000-0000-0000-000000001000";
const char* CHBT_UUID = "00000000-0000-0000-0000-000000010000";

int charMode = 1;
const char* myLocalName = "ArduinoBLE";
BLEService myBLEService(serviceUUID);

BLEIntCharacteristic CHAR_CHLH(CHLH_UUID, BLERead | BLEWrite);
BLEIntCharacteristic CHAR_CHRH(CHRH_UUID, BLERead | BLEWrite);
BLEIntCharacteristic CHAR_CHBT(CHBT_UUID, BLERead | BLEWrite);

signed int BLE_Vx, BLE_Vy, BLE_Cx, BLE_Cy, BLE_Sw;

#define pi 3.1416
#define Rd 39.5
#define L1 118.5
#define L2 82.5
#define SL 70.6549
#define Nmmps 142
#define Nrps .5722

// X = 82.5 <> -82.5
// Y = 118.5 <> -118.5

#define pinMotRFpwm 2
#define pinMotRF2 4
#define pinMotRF1 3

#define pinMotLF1 6
#define pinMotLF2 5
#define pinMotLFpwm 7

#define pinMotRBpwm 8
#define pinMotRB1 9
#define pinMotRB2 10

#define pinMotLBpwm 11
#define pinMotLB2 A1
#define pinMotLB1 A0

#define pwmMaxLimit 255

#define pinLED A2
#define pinComMode 12
#define mmPerPulse .pwmMin



bool bleComDeactivated = true;

int ppr1 = 8670, ppr2 = 8560, ppr3 = 8598, ppr4 = 8612;
int mmpp1 = 0, mmpp2 = 0, mmpp3 = 0, mmpp4 = 0;

int i, j;

signed int TargEncSpeed[4] = { 0, 0, 0, 0 };
signed int PastEncSpeed[4] = { 0, 0, 0, 0 };
signed int PresEncSpeed[4] = { 0, 0, 0, 0 };
signed int pastError[4] = { 0, 0, 0, 0 };

signed int PWMSpeed[4] = { 0, 0, 0, 0 };
signed int VX, VY, W0;
signed int Prev_Ser_Vx = 0, Prev_Ser_Vy = 0;
float Prev_Ser_W0 = 0.0;
int currentSpeed = 20;


signed int W1 = 0, W2 = 0, W3 = 0, W4 = 0;
int velo;
float ang;
signed int errors = 0, derivative = 0, integral = 0, pid = 0;
bool motorPoleReverse = true;
// kp = .06, kd = .2


class Velocity {
public:
  int Ser_Vx = 0;
  int Ser_Vy = 0;
  float Ser_W0 = 0.00;
};

int Ser_Vx = 0;
int Ser_Vy = 0;
float Ser_W0 = 0.00;

void initPinModeOutput(int pinFirst, int pinLast) {
  for (i = pinFirst; i <= pinLast; i++) {
    pinMode(i, OUTPUT);
  }
  pinMode(A0, OUTPUT);
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
}

void initPinModeInput(int pinFirst, int pinLast) {
  for (i = pinFirst; i <= pinLast; i++) {
    pinMode(i, INPUT);
  }
}
void pinModeDef1() {
  // for (i = 0; i < 4; i++) {
  //   pinMode(pinIntV + i, INPUT);
  // }
}

void printSerialData(int Ser_Vx, int Ser_Vy, float Ser_W0) {
  // Serial.print(Ser_Vx);
  // Serial.print(" <> ");
  // Serial.print(Ser_Vy);
  // Serial.print(" <> ");
  // Serial.print(Ser_W0);
  // Serial.print(" ");
  // Serial.print(" | ");
}

void printSerial1EncoderData() {

  // Serial.print(W1);
  // Serial.print("\t");
  // Serial.print(W2);
  // Serial.print("\t");
  // Serial.print(W3);
  // Serial.print("\t");
  // Serial.print(W4);
  // Serial.print("\n");
}

void splitSerialData(String strData, const char strSep) {
  unsigned int str_len = strData.length() + 1;
  char char_array[str_len];
  strData.toCharArray(char_array, str_len);
  Ser_Vx = int(atof(strtok(char_array, ",")));               //int(atof(strtok(char_array, ","))) * sin(ang);
  Ser_Vy = int(atof(strtok(strtok(char_array, ","), ",")));  //int(atof(strtok(char_array, ","))) * cos(ang);
  Ser_W0 = float(atof(strtok(NULL, ",")));
  // getMotorsSpeed(Ser_Vx, Ser_Vy, Ser_W0);

  // runAllMotorFromSpeed();
}





void setup() {
  // Wire.begin(1);
  // Wire.onReceive(checkI2CEncData);

  Serial.begin(115200);
  Serial1.begin(115200);
  myTransfer.begin(Serial1);

  initPinModeOutput(pinMotLF1, 11);
  pinModeDef1();

  pinMode(pinComMode, INPUT_PULLDOWN);

  // Serial.print("pinComMode : ");
  // Serial.println(digitalRead(pinComMode));
  // if (digitalRead(pinComMode) == HIGH) {
  //   bleComDeactivated = false;
  //   startBLEService();
  //   delay(2000);
  //   delay(200);
  //   threadBLE.start(mbed::callback(readBLEData));
  // } else {
  //   bleComDeactivated = true;
  // }

  PT_INIT(&runMot1);
  PT_INIT(&runMot2);
  PT_INIT(&runMot3);
  PT_INIT(&runMot4);
  PT_INIT(&brakeMot);
  delay(3000);
  // Serial.print("Connected....");
}


int serRun = 0;


void stringSplit(String serialData) {

  int dataArrSize = 3;
  char delimiter = ',';
  String dataArr[3];

  int currentIndex = 0;
  int nextIndex = serialData.indexOf(delimiter);
  int i = 0;



  while (nextIndex != -1 && i < dataArrSize) {
    if (i == 0)
      Ser_Vx = serialData.substring(currentIndex, nextIndex).toInt();
    else if (i == 1)
      Ser_Vy = serialData.substring(currentIndex, nextIndex).toInt();
    else if (i == 2)
      Ser_W0 = atof(serialData.substring(currentIndex, nextIndex).c_str());

    currentIndex = nextIndex + 1;
    nextIndex = serialData.indexOf(delimiter, currentIndex);
    i++;
  }

  if (currentIndex != serialData.length() && i < dataArrSize) {
    if (currentIndex == 1) {
      Ser_Vy = 0;
      Ser_W0 = 0.0;
    } else if (currentIndex == 2) {
      Ser_W0 = 0.0;
    } else {
      Ser_Vx = 0;
      Ser_Vy = 0;
      Ser_W0 = 0.0;
    }
  }
}


char separ = ',';

void SerialDataReceived(char* line) {
  Serial.println(line);
  // serialData = Serial.readStringUntil('\n');

  // Serial.println();
  char separ = ',';
  StringSplitter* splitter = new StringSplitter(line, separ, 3);
  int itemCount = splitter->getItemCount();
  int Ser_Vx = (splitter->getItemAtIndex(0)).toInt();
  int Ser_Vy = (splitter->getItemAtIndex(1)).toInt();
  float Ser_W0 = (splitter->getItemAtIndex(2)).toFloat();

  if (Ser_Vx == 0 && Ser_Vy == 0 && Ser_W0) {
    brake();
  } else {
    getMotorsSpeed(Ser_Vx, Ser_Vy, Ser_W0);
    // pidControl();
    if (Prev_Ser_Vx != Ser_Vx && Prev_Ser_Vy != Ser_Vy && Prev_Ser_W0 != Ser_W0) {
      runAllMotorFromSpeed();
      Prev_Ser_Vx = Ser_Vx;
      Prev_Ser_Vy = Ser_Vy;
      Prev_Ser_W0 = Ser_W0;
    }
  }
  // checkSerial1EncData();
  // printPWMSpeed();
}




void loop() {
  // while(!Serial.available()){
  //   delay(1);
  // }

  if (Serial.available()) {
    // reader.poll();
    // if (reader.available()) {
    //   char serialData[reader.len()];
    //   reader.read(serialData);
    String serialData = Serial.readStringUntil('\n');

    // Serial.print(serialData);
    // Serial.print(" | ");
    // splitSerialData(String(serialData), separ);
    stringSplit(String(serialData));
    Serial.print(Ser_Vx);
    Serial.print(", ");
    Serial.print(Ser_Vy);
    Serial.print(", ");
    Serial.print(Ser_W0);
    Serial.println(" <|> ");
    // Serial.print(Prev_Ser_Vx);
    // Serial.print(", ");
    // Serial.print(Prev_Ser_Vy);
    // Serial.print(", ");
    // Serial.print(Prev_Ser_W0);
    // Serial.println(" | ");


    if (Ser_Vx == 0 && Ser_Vy == 0 && Ser_W0 == 0.0) {
      brake();
      getMotorsSpeed(Ser_Vx, Ser_Vy, Ser_W0);
      Prev_Ser_Vx = Ser_Vx;
      Prev_Ser_Vy = Ser_Vy;
      Prev_Ser_W0 = Ser_W0;

    } else if (Prev_Ser_Vx != Ser_Vx || Prev_Ser_Vy != Ser_Vy || Prev_Ser_W0 != Ser_W0) {
      // Serial.println("<<<getMotorsSpeed >>>");
      getMotorsSpeed(Ser_Vx, Ser_Vy, Ser_W0);
      runAllMotorFromSpeed();
      Prev_Ser_Vx = Ser_Vx;
      Prev_Ser_Vy = Ser_Vy;
      Prev_Ser_W0 = Ser_W0;
    }

    // printPWMSpeed();
  }

//  else {
//    pidControl();
//  }

  delay(delayTime - 1);
  // serRun++;
}






void checkSerial1EncData() {
  uint16_t recSize = 0;
  if (myTransfer.available()) {
    recSize = myTransfer.rxObj(PresEncSpeed[0], recSize);
    recSize = myTransfer.rxObj(PresEncSpeed[1], recSize);
    recSize = myTransfer.rxObj(PresEncSpeed[2], recSize);
    recSize = myTransfer.rxObj(PresEncSpeed[3], recSize);
  }
}
bool calcPID = true;

int calculatePid(int id) {
  errors = TargEncSpeed[id] - PresEncSpeed[id];
  derivative = (errors - pastError[id]) * 1000 / delayTime;
  integral = (errors * delayTime / 1000) + pastError[id];
  pid = int(kp[id] * errors + kd[id] * derivative + ki[id] * integral);
  // Serial.print(pid);
  // Serial.print(" |\t");
  pastError[id] = errors;
  return pid;
}

void pidControl() {
  uint16_t recSize = 0;
  

  if (myTransfer.available()) {
    recSize = myTransfer.rxObj(PresEncSpeed[0], recSize);
    recSize = myTransfer.rxObj(PresEncSpeed[1], recSize);
    recSize = myTransfer.rxObj(PresEncSpeed[2], recSize);
    recSize = myTransfer.rxObj(PresEncSpeed[3], recSize);

    // if (TargEncSpeed[0] == 0 && TargEncSpeed[1] == 0 && TargEncSpeed[2] == 0 && TargEncSpeed[3] == 0) {
    //   brake();
    //   if (abs(PresEncSpeed[0]) < 1 && abs(PresEncSpeed[1]) < 1 && abs(PresEncSpeed[2]) < 3 && abs(PresEncSpeed[3]) < 3) {
    //     pastError[0] = 0;
    //     pastError[1] = 0;
    //     pastError[2] = 0;
    //     pastError[3] = 0;
    //   }
    
    if (TargEncSpeed[0] == 0 && TargEncSpeed[1] == 0 && TargEncSpeed[2] == 0 && TargEncSpeed[3] == 0) {
      brake();
    }
    else if ( abs(PresEncSpeed[0]) < 5 && abs(PresEncSpeed[1]) < 5 && abs(PresEncSpeed[2]) < 5 && abs(PresEncSpeed[3]) < 5){
    }

    else if (abs(TargEncSpeed[0] - PresEncSpeed[0]) > 20 && abs(TargEncSpeed[1] - PresEncSpeed[1]) > 20 && abs(TargEncSpeed[2] - PresEncSpeed[2]) > 20 && abs(TargEncSpeed[3] - PresEncSpeed[3]) > 20) {
    }

  } 

  else if (calcPID == true) {
    PWMSpeed[0] = checkMaxSpeed(PWMSpeed[0] + calculatePid(0));
    PWMSpeed[1] = checkMaxSpeed(PWMSpeed[1] + calculatePid(1));
    PWMSpeed[2] = checkMaxSpeed(PWMSpeed[2] + calculatePid(2));
    PWMSpeed[3] = checkMaxSpeed(PWMSpeed[3] + calculatePid(3));
    runAllMotorFromSpeed();
    
    printPWMSpeed();
  }
  // PastEncSpeed[0] = PresEncSpeed;
  

  // printPWMSpeed();
}


void CheckEncoderValues(int dir) {
  int sp = 40;
  if (dir == 0) {
    runMotorTimes(sp, 0, 0);
  } else if (dir == 3) {
    runMotorTimes(0, sp, 0);
  } else if (dir == 2) {
    runMotorTimes(-sp, 0, 0);
  } else if (dir == 1) {
    runMotorTimes(0, -sp, 0);
  }
}



void runMotorTimes(signed int x, signed int y, signed int w) {
}

void dataReceive(int bt) {
  curTime = millis();
  char svx = Wire.read();
  int vx = Wire.read();
  char svy = Wire.read();
  int vy = Wire.read();
  char swo = Wire.read();
  int wo = Wire.read();

  if (svx == '+') {
    VX = vx;
  } else {
    VX = -vx;
  }

  if (svy == '+') {
    VY = vy;
  } else {
    VY = -vy;
  }

  if (swo == '+') {
    W0 = wo;
  } else {
    W0 = -wo;
  }

  if (VX == 0 && VY == 0 && W0 == 0) {
    brake();
  } else {
    getMotorsSpeed(VX, VY, W0);
    // Serial.print(VX);
    // Serial.print("\t");
    // Serial.print(VY);
    // Serial.print("\t");
    // Serial.print(W0);
    // Serial.print("\n");

    runAllMotorFromSpeed();
  }
  // Serial.println(millis() - curTime);
  // delay(80);
}

void printReceivedData() {
  // Serial.print(VX);
  // Serial.print("   ");
  // Serial.print(VY);
  // Serial.print("   ");
  // Serial.print(W0);
  // Serial.print("   ");
}

void startBLEService() {
  if (!BLE.begin()) {
    // Serial.println("Starting BLE failed!");
    while (1)
      ;
  }

  for (i = 0; i < 5; i++) {
    analogWrite(pinLED, 0);
    delay(100);
    analogWrite(pinLED, 1000);
    delay(100);
  }

  // Serial.println("BLE initialized.");
  // Serial.print("MAC: ");
  // Serial.println(BLE.address());
  // Serial.println("Service UUIID: \t\t\t" + String(serviceUUID));

  // Serial.println();
  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

  BLE.setLocalName(myLocalName);
  BLE.setAdvertisedService(myBLEService);
  myBLEService.addCharacteristic(CHAR_CHLH);
  // myBLEService.addCharacteristic(CHAR_CHLV);
  myBLEService.addCharacteristic(CHAR_CHRH);
  // myBLEService.addCharacteristic(CHAR_CHRV);
  myBLEService.addCharacteristic(CHAR_CHBT);
  BLE.addService(myBLEService);
  BLE.advertise();

  // Serial.println("Initialized BLE Done...");
}

void blePeripheralConnectHandler(BLEDevice central) {
  // central connected event handler
  // Serial.print("Connected event, central: ");
  // Serial.println(central.address());
}

void blePeripheralDisconnectHandler(BLEDevice central) {
  // central disconnected event handler
  // Serial.print("Disconnected event, central: ");
  // Serial.println(central.address());
}

void readBLEData(void) {
  int delayRotation = 50;

  for (;;) {
    BLEDevice central = BLE.central();


    digitalWrite(pinLED, HIGH);
    delay(100);
    digitalWrite(pinLED, LOW);
    delay(100);

    if (central) {
      digitalWrite(pinLED, HIGH);
      delay(1);
      // Serial.print("Connected: ");
      // Serial.println(central.address());

      while (central.connected()) {
        if (CHAR_CHLH.written()) {
          int tempVal = CHAR_CHLH.value();

          BLE_Vy = int(currentSpeed * cos(double(tempVal) * 3.1416 / 90));
          BLE_Vx = int(currentSpeed * sin(double(tempVal) * 3.1416 / 90));
          if (tempVal > 180 and tempVal <= 270) {
            BLE_Vx = -BLE_Vx;
          } else if (tempVal > 270 and tempVal <= 3600) {
            BLE_Vy = -BLE_Vy;
          }
          // Serial.print("CHAR_CHLH received: ");
          // Serial.println(tempVal);
          getMotorsSpeed(BLE_Vx, BLE_Vy, 0);

          runAllMotorFromSpeed();
          CHAR_CHLH.writeValue(tempVal);
        }
        if (CHAR_CHRH.written()) {
          int tempVal = CHAR_CHRH.value();
          BLE_Cx = CHAR_CHRH.value();
          // Serial.print("CHAR_CHRH received: ");
          // Serial.println(tempVal);
          CHAR_CHRH.writeValue(tempVal);
        }
        if (CHAR_CHBT.written()) {
          BLE_Vy = 0;
          BLE_Vx = 0;

          int tempVal = CHAR_CHBT.value();

          if (tempVal == 0) {
            BLE_Sw = 0;
            brake();
            delay(30);
          } else if (tempVal >= 30 && tempVal <= 100) {
            currentSpeed = int(currentSpeed * tempVal / 50);
          } else if (tempVal == 3) {
            getMotorsSpeed(0, 0, 1);

            runAllMotorFromSpeed();
            delay(delayRotation);
            brake();
          } else if (tempVal == 4) {
            getMotorsSpeed(0, 0, -1);

            runAllMotorFromSpeed();
            delay(delayRotation);
            brake();
          } else if (tempVal >= 8 && tempVal <= 11) {
            CheckEncoderValues(tempVal - 8);
          } else {
            BLE_Sw = 0;
            brake();
            delay(30);
          }
          CHAR_CHBT.writeValue(tempVal);
          // Serial.print("CHAR_CHBT received: ");
          // Serial.println(tempVal);
        }
      }

      // Serial.println("Disconnected.");
      BLE_Vx = 0;
      BLE_Vy = 0;
      BLE_Sw = 0;
    }
  }
}


void runAllMotorFromSpeed() {
  runMotor(3, PWMSpeed[3]);
  runMotor(2, PWMSpeed[2]);
  runMotor(0, PWMSpeed[0]);
  runMotor(1, PWMSpeed[1]);
  // for (i = 0; i < 4; i++) {
  //   runMotor(i, PWMSpeed[i]);
  // }
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
  analogWrite(pinMotLBpwm, 240);
  digitalWrite(pinMotRB1, HIGH);
  digitalWrite(pinMotRB2, HIGH);
  analogWrite(pinMotRBpwm, 240);
  digitalWrite(pinMotLF1, HIGH);
  digitalWrite(pinMotLF2, HIGH);
  analogWrite(pinMotLFpwm, 240);
  digitalWrite(pinMotRF1, HIGH);
  digitalWrite(pinMotRF2, HIGH);
  analogWrite(pinMotRFpwm, 240);
}


void brake() {
  // Serial.println("Braking");
  TargEncSpeed[0] = 0;
  TargEncSpeed[1] = 0;
  TargEncSpeed[2] = 0;
  TargEncSpeed[3] = 0;
  PT_SCHEDULE(brakeThread(&brakeMot));
}


signed int checkMaxSpeed(signed int curSpeed) {
  if (curSpeed < 0) {
    if (abs(curSpeed) > pwmMaxLimit) {
      return -pwmMaxLimit;
    } else {
      return curSpeed;
    }
  } else {
    if (abs(curSpeed) > pwmMaxLimit) {
      return pwmMaxLimit;
    } else {
      return curSpeed;
    }
  }
}

float ap1 = -4096.39516, bp1 = 130.63112, an1 = -4069.71299, bn1 = -129.32008, ap2 = -4446.28504, bp2 = 132.61027, an2 = -4565.02843, bn2 = -133.68403, ap3 = -4431.14923, bp3 = 137.10841, an3 = -4060.2701, bn3 = -130.08781, ap4 = -3954.66101, bp4 = 129.7428, an4 = -4147.72985, bn4 = -132.54867;
int pwmMin = 50;

void getMotorsSpeed(int Vx, int Vy, float W0) {
  Serial.println("getMotorsSpeed");
  if (Vx == 1628 && Vy == 1628 && W0 == 1628) {
    int WP = 1200;
    PWMSpeed[0] = 254;  //int(ap1 / (WP - bp1));
    PWMSpeed[1] = 254;  //int(ap2 / (WP - bp2));
    PWMSpeed[2] = 254;  //int(ap3 / (WP - bp3));
    PWMSpeed[3] = 254;  //int(ap4 / (WP - bp4));
  }

  else if (Vx == -1628 && Vy == -1628 && W0 == -1628) {
    int WN = -1150;
    PWMSpeed[0] = -254;  //int(an1 / (WN - bn1));
    PWMSpeed[1] = -254;  //int(an2 / (WN - bn2));
    PWMSpeed[2] = -254;  //int(an3 / (WN - bn3));
    PWMSpeed[3] = -254;  //int(an4 / (WN - bn4));
  }

  else {
    PWMSpeed[0] = int(Vx + Vy - (W0 * (L1 + L2)));

    if (PWMSpeed[0] >= pwmMin) {
      TargEncSpeed[0] = PWMSpeed[0];
      PWMSpeed[0] = int(ap1 / (PWMSpeed[0] - bp1));
    } else if (PWMSpeed[0] <= -pwmMin) {
      TargEncSpeed[0] = PWMSpeed[0];
      PWMSpeed[0] = int(an1 / (PWMSpeed[0] - bn1));
    } else {
      PWMSpeed[0] = 0;
      TargEncSpeed[0] = PWMSpeed[0];
    }

    PWMSpeed[1] = int(Vx - Vy + (W0 * (L1 + L2)));

    if (PWMSpeed[1] >= pwmMin) {
      TargEncSpeed[1] = PWMSpeed[1];
      PWMSpeed[1] = int(ap2 / (PWMSpeed[1] - bp2));
    } else if (PWMSpeed[1] <= -pwmMin) {
      TargEncSpeed[1] = PWMSpeed[1];
      PWMSpeed[1] = int(an2 / (PWMSpeed[1] - bn2));
    } else {
      PWMSpeed[1] = 0;
      TargEncSpeed[1] = PWMSpeed[1];
    }

    PWMSpeed[2] = int(Vx - Vy - (W0 * (L1 + L2)));

    if (PWMSpeed[2] >= pwmMin) {
      TargEncSpeed[2] = PWMSpeed[2];
      PWMSpeed[2] = int(ap3 / (PWMSpeed[2] - bp3));
    } else if (PWMSpeed[2] <= -pwmMin) {
      TargEncSpeed[2] = PWMSpeed[2];
      PWMSpeed[2] = int(an3 / (PWMSpeed[2] - bn3));
    } else {
      PWMSpeed[2] = 0;
      TargEncSpeed[2] = PWMSpeed[2];
    }

    PWMSpeed[3] = int(Vx + Vy + (W0 * (L1 + L2)));

    if (PWMSpeed[3] >= pwmMin) {
      TargEncSpeed[3] = PWMSpeed[3];
      PWMSpeed[3] = int(ap4 / (PWMSpeed[3] - bp4));
    } else if (PWMSpeed[3] <= -pwmMin) {
      TargEncSpeed[3] = PWMSpeed[3];
      PWMSpeed[3] = int(an4 / (PWMSpeed[3] - bn4));
    } else {
      PWMSpeed[3] = 0;
      TargEncSpeed[3] = PWMSpeed[3];
    }
  }


  for (i = 0; i < 4; i++) {
    int motorSpeed = checkMaxSpeed(PWMSpeed[i]);
    PWMSpeed[i] = motorSpeed;
  }

  runMotor(3, PWMSpeed[3]);
  runMotor(2, PWMSpeed[2]);
  runMotor(0, PWMSpeed[0]);
  runMotor(1, PWMSpeed[1]);

  for (i = 10; i < 10; i++) {
    delay(20);
    checkSerial1EncData();
    // Serial.print(i);
    // Serial.print(" :\t");
    // printPWMSpeed();
  }
}

signed int ReluSpeed(signed int PWMSpeedss, float Ratio) {
  if (PWMSpeedss > 0 && int(PWMSpeedss * Ratio) > pwmMin) {
    return PWMSpeedss * Ratio;
  } else if (PWMSpeedss > 0 && int(PWMSpeedss * Ratio) < -pwmMin) {
    return PWMSpeedss * Ratio;
  } else {
    return PWMSpeedss;
  }
}


void printPWMSpeed() {
  // if (PresEncSpeed[0] != 0 || PresEncSpeed[1] != 0 || PresEncSpeed[2] || 0 && PresEncSpeed[3] || 0) {

  // Serial.print("ER : ");
  // Serial.print(pastError[0]);
  // Serial.print(",");
  // Serial.print(pastError[1]);
  // Serial.print(",");
  // Serial.print(pastError[2]);
  // Serial.print(",");
  // Serial.print(pastError[3]);
  // Serial.print("   |   ");

  // Serial.print("TS : ");
  // Serial.print(TargEncSpeed[0]);
  // Serial.print(",");
  // Serial.print(TargEncSpeed[1]);
  // Serial.print(",");
  // Serial.print(TargEncSpeed[2]);
  // Serial.print(",");
  // Serial.print(TargEncSpeed[3]);
  // Serial.print("   |   ");

  Serial.print("Spd : ");
  Serial.print(TargEncSpeed[0]);
  Serial.print(",  ");
  Serial.print(PresEncSpeed[0]);
  Serial.print(",");
  Serial.print(PresEncSpeed[1]);
  Serial.print(",");
  Serial.print(PresEncSpeed[2]);
  Serial.print(",");
  Serial.print(PresEncSpeed[3]);
  Serial.print("   |   ");

  // Serial.print("MS : ");
  // Serial.print(TargEncSpeed[0]);
  // Serial.print(",");
  // Serial.print(PWMSpeed[0]);
  // Serial.print(",");
  // Serial.print(PWMSpeed[1]);
  // Serial.print(",");
  // Serial.print(PWMSpeed[2]);
  // Serial.print(",");
  // Serial.print(PWMSpeed[3]);

  Serial.println();
  // }
}
