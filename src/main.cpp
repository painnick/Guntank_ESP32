#include <Arduino.h>

#include <Ps3Controller.h>
#include <ESP32Servo.h>

#include "soc/soc.h"             // disable brownout problems
#include "soc/rtc_cntl_reg.h"    // disable brownout problems

// These are all GPIO pins on the ESP32
// Recommended pins include 2,4,12-19,21-23,25-27,32-33
// for the ESP32-S2 the GPIO pins are 1-21,26,33-42
#define PIN_LEFT_ARM 15
#define PIN_RIGHT_ARM 16
#define PIN_BODY 14

Servo servoBody;
Servo servoLeftArm, servoRightArm;

int center = 90;

int bodyAngle = center;
int leftArmAngle = center;
int rightArmAngle = center;

int angleStep = 10;
int servoDelay = 15;

void init() {
  bodyAngle = center;
  leftArmAngle = center;
  rightArmAngle = center;

  servoBody.write(bodyAngle);
  servoLeftArm.write(leftArmAngle);
  servoRightArm.write(rightArmAngle);
}

void notify()
{
  if (Ps3.event.button_down.start) {
    init();
  }

  if (Ps3.event.analog_changed.button.l1) {
    Serial.println("L1");
    leftArmAngle = min(leftArmAngle + angleStep, 180);
    servoLeftArm.write(leftArmAngle);
    delay(servoDelay);
  }
  if (Ps3.event.analog_changed.button.l2) {
    Serial.println("L2");
    leftArmAngle = max(leftArmAngle - angleStep, 0);
    servoLeftArm.write(leftArmAngle);
    delay(servoDelay);
  }
  if (Ps3.event.analog_changed.button.r1) {
    Serial.println("R1");
    rightArmAngle = min(rightArmAngle + angleStep, 180);
    servoRightArm.write(rightArmAngle);
    delay(servoDelay);
  }
  if (Ps3.event.analog_changed.button.r2) {
    Serial.println("R2");
    rightArmAngle = max(rightArmAngle - angleStep, 0);
    servoRightArm.write(rightArmAngle);
    delay(servoDelay);
  }

  if (Ps3.event.analog_changed.button.left) {
    Serial.println("Left");
    bodyAngle = max(bodyAngle - 1, 0);
    servoBody.write(bodyAngle);
    delay(servoDelay);
  }
  if (Ps3.event.analog_changed.button.right) {
    Serial.println("Right");
    bodyAngle = min(bodyAngle + 1, 180);
    servoBody.write(bodyAngle);
    delay(servoDelay);
  }
}

void onConnect() {
  Ps3.setPlayer(0);
  Serial.println("Set Player #0");
}

void setup()
{
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector

  Serial.begin(115200);
  Ps3.attach(notify);
  Ps3.attachOnConnect(onConnect);
  Ps3.begin("44:44:44:44:44:44");

  while(!Ps3.isConnected()) {
    delay(100);
  }
  String address = Ps3.getAddress();

  Serial.print("The ESP32's Bluetooth MAC address is: ");
  Serial.println(address);

  Ps3.setPlayer(0);

  servoBody.attach(PIN_BODY, 500, 2400);
  servoLeftArm.attach(PIN_LEFT_ARM, 500, 2400);
  servoRightArm.attach(PIN_RIGHT_ARM, 500, 2400);

  init();
}

void loop()
{
}
