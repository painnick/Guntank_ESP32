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

int bodyAngle = 0;
int leftArmAngle = 0;
int rightArmAngle = 0;

void notify()
{
  if(Ps3.data.button.circle > 0) {
    Serial.println("Body!");
    bodyAngle = (bodyAngle + 1) % 180;
    servoBody.write(bodyAngle);
    delay(20);
  }
  if(Ps3.data.button.triangle > 0) {
    Serial.println("Left Atm!");
    leftArmAngle = (leftArmAngle + 1) % 180;
    servoLeftArm.write(leftArmAngle);
    delay(20);
  }
  if(Ps3.data.button.cross > 0) {
    Serial.println("Right Atm!");
    rightArmAngle = (rightArmAngle + 1) % 180;
    servoRightArm.write(rightArmAngle);
    delay(20);
  }
}

void setup()
{
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector

  Serial.begin(115200);
  Ps3.attach(notify);
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

  servoBody.write(0);
  servoLeftArm.write(0);
  servoRightArm.write(0);
}


void loop()
{
  delay(1000);
  Ps3.setRumble(100, 500);
}
