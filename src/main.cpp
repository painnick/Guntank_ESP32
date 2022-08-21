#include <Arduino.h>

#include <Ps3Controller.h>
#include <ESP32Servo.h>

#include "soc/soc.h"             // disable brownout problems
#include "soc/rtc_cntl_reg.h"    // disable brownout problems

// These are all GPIO pins on the ESP32
// Recommended pins include 2,4,12-19,21-23,25-27,32-33
// for the ESP32-S2 the GPIO pins are 1-21,26,33-42
#define PIN_LEFT_ARM 17
#define PIN_RIGHT_ARM 16
#define PIN_BODY 14

#define PIN_LEFT_GUN 12
#define PIN_RIGHT_GUN 13

#define PIN_TRACK_A1 32
#define PIN_TRACK_A2 33
#define PIN_TRACK_B1 25
#define PIN_TRACK_B2 26

#define CHANNEL_A1 12
#define CHANNEL_A2 13
#define CHANNEL_B1 14
#define CHANNEL_B2 15

Servo servoBody;
Servo servoLeftArm, servoRightArm;

int center = 90;

int bodyAngle = center;
int leftArmAngle = center;
int rightArmAngle = center;

int angleStep = 5;
int servoDelay = 15;

bool circlePress = false;
bool triaglePress = false;
bool squarePress = false;
bool crossPress = false;

void init() {
  bodyAngle = center;
  leftArmAngle = center;
  rightArmAngle = center;

  servoBody.write(bodyAngle);
  servoLeftArm.write(leftArmAngle);
  servoRightArm.write(rightArmAngle);
}

void fire(int pin) {
  digitalWrite(pin, HIGH);
  delay(100);
  digitalWrite(pin, LOW);
}

int battery = 0;
void notify()
{
  if (Ps3.event.button_down.start) {
    init();
  }

  if (Ps3.event.button_down.cross) {
    fire(PIN_LEFT_GUN);
  }
  if (Ps3.event.button_down.circle) {
    fire(PIN_RIGHT_GUN);
  }

  if (Ps3.event.analog_changed.button.l1) {
    Serial.println("L1");
    leftArmAngle = max(leftArmAngle - angleStep, center - 45);
    servoLeftArm.write(leftArmAngle);
  }
  if (Ps3.event.analog_changed.button.l2) {
    Serial.println("L2");
    leftArmAngle = min(leftArmAngle + angleStep, center + 45);
    servoLeftArm.write(leftArmAngle);
  }
  if (Ps3.event.analog_changed.button.r1) {
    Serial.println("R1");
    rightArmAngle = min(rightArmAngle + angleStep, center + 45);
    servoRightArm.write(rightArmAngle);
  }
  if (Ps3.event.analog_changed.button.r2) {
    Serial.println("R2");
    rightArmAngle = max(rightArmAngle - angleStep, center - 45);
    servoRightArm.write(rightArmAngle);
  }

  if (Ps3.event.analog_changed.button.left) {
    Serial.println("Left");
    bodyAngle = min(bodyAngle + 5, 180);
    servoBody.write(bodyAngle);
  }
  if (Ps3.event.analog_changed.button.right) {
    Serial.println("Right");
    bodyAngle = max(bodyAngle - 5, 0);
    servoBody.write(bodyAngle);
  }

  int absLy = abs(Ps3.event.analog_changed.stick.ly);
  if (absLy < 10) {
    ledcWrite(CHANNEL_B1, 0);
    ledcWrite(CHANNEL_B2, 0);
  } else {
    if (Ps3.event.analog_changed.stick.ly < -10) {
      ledcWrite(CHANNEL_B1, absLy);
      ledcWrite(CHANNEL_B2, 0);
    }
    else if (Ps3.event.analog_changed.stick.ly > 10) {
      ledcWrite(CHANNEL_B1, 0);
      ledcWrite(CHANNEL_B2, absLy);
    }
  }

  int absRy = abs(Ps3.event.analog_changed.stick.ry);
  if (absRy < 10) {
    ledcWrite(CHANNEL_A1, 0);
    ledcWrite(CHANNEL_A2, 0);    
  } else {
    if (Ps3.event.analog_changed.stick.ry < -10) {
      ledcWrite(CHANNEL_A1, absRy);
      ledcWrite(CHANNEL_A2, 0);
    }
    else if (Ps3.event.analog_changed.stick.ry > 10) {
      ledcWrite(CHANNEL_A1, 0);
      ledcWrite(CHANNEL_A2, absRy);
    }
  }
}

void onConnect();
void onDisconnect();

void initPs3() {
  Ps3.attach(notify);
  Ps3.attachOnConnect(onConnect);
  Ps3.attachOnDisconnect(onDisconnect);
  Ps3.begin("44:44:44:44:44:44");
}

void onConnect() {
  String address = Ps3.getAddress();

  Serial.print("The ESP32's Bluetooth MAC address is: ");
  Serial.println(address);

  init();

  servoBody.attach(PIN_BODY, 500, 2400);
  servoLeftArm.attach(PIN_LEFT_ARM, 500, 2400);
  servoRightArm.attach(PIN_RIGHT_ARM, 500, 2400);

  pinMode(PIN_LEFT_GUN, OUTPUT);
  pinMode(PIN_RIGHT_GUN, OUTPUT);


  ledcSetup(CHANNEL_A1, 5000, 7); // 0~127
  ledcSetup(CHANNEL_A2, 5000, 7); // 0~127
  ledcSetup(CHANNEL_B1, 5000, 7); // 0~127
  ledcSetup(CHANNEL_B2, 5000, 7); // 0~127

  // pinMode(PIN_TRACK_A1, OUTPUT);
  // pinMode(PIN_TRACK_A2, OUTPUT);
  // pinMode(PIN_TRACK_B1, OUTPUT);
  // pinMode(PIN_TRACK_B2, OUTPUT);
  ledcAttachPin(PIN_TRACK_A1, CHANNEL_A1);
  ledcAttachPin(PIN_TRACK_A2, CHANNEL_A2);
  ledcAttachPin(PIN_TRACK_B1, CHANNEL_B1);
  ledcAttachPin(PIN_TRACK_B2, CHANNEL_B2);

  init();
}

void onDisconnect() {
  Ps3.end();
  initPs3();
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
  Serial.begin(115200);
  init();
  initPs3();
}

void loop() {
  if(!Ps3.isConnected())
    return;
}
