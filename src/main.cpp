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

#define PIN_LEFT_GUN 12
#define PIN_RIGHT_GUN 13

#define PIN_TRACK_A1 32
#define PIN_TRACK_A2 33
#define PIN_TRACK_B1 25
#define PIN_TRACK_B2 26


Servo servoBody;
Servo servoLeftArm, servoRightArm;

int center = 90;

int bodyAngle = center;
int leftArmAngle = center;
int rightArmAngle = center;

int angleStep = 10;
int servoDelay = 15;

bool leftTrackrunning = false;
bool rightTrackrunning = false;

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

  if (Ps3.event.button_down.l3) {
    fire(PIN_LEFT_GUN);
  }
  if (Ps3.event.button_down.r3) {
    fire(PIN_RIGHT_GUN);
  }

  if (Ps3.event.analog_changed.button.l1) {
    Serial.println("L1");
    leftArmAngle = min(leftArmAngle + angleStep, 180);
    servoLeftArm.write(90 - leftArmAngle);
    delay(servoDelay);
  }
  if (Ps3.event.analog_changed.button.l2) {
    Serial.println("L2");
    leftArmAngle = max(leftArmAngle - angleStep, 0);
    servoLeftArm.write(90 - leftArmAngle);
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

  if (Ps3.event.button_down.l3) {
    digitalWrite(PIN_TRACK_A1, LOW);
    digitalWrite(PIN_TRACK_A2, LOW);
  }
  if (Ps3.event.button_down.r3) {
    digitalWrite(PIN_TRACK_B1, LOW);
    digitalWrite(PIN_TRACK_B2, LOW);
  }

  if (Ps3.event.button_down.triangle) {
    digitalWrite(PIN_TRACK_A1, HIGH);
    digitalWrite(PIN_TRACK_A2, LOW);
    leftTrackrunning = true;
  } else if (Ps3.event.button_down.square) {
    digitalWrite(PIN_TRACK_A1, LOW);
    digitalWrite(PIN_TRACK_A2, HIGH);
    leftTrackrunning = true;
  } else if (leftTrackrunning && !Ps3.event.button_down.triangle && !Ps3.event.button_down.square) {
    digitalWrite(PIN_TRACK_A1, LOW);
    digitalWrite(PIN_TRACK_A2, LOW);
    leftTrackrunning = false;
  }

  if (Ps3.event.button_down.circle) {
    digitalWrite(PIN_TRACK_B1, HIGH);
    digitalWrite(PIN_TRACK_B2, LOW);
    rightTrackrunning = true;
  } else if (Ps3.event.button_down.cross) {
    digitalWrite(PIN_TRACK_B1, LOW);
    digitalWrite(PIN_TRACK_B2, HIGH);
    rightTrackrunning = true;
  } else if (rightTrackrunning && !Ps3.event.button_down.circle && !Ps3.event.button_down.cross) {
    digitalWrite(PIN_TRACK_B1, LOW);
    digitalWrite(PIN_TRACK_B2, LOW);
    rightTrackrunning = false;
  }

  // if( battery != Ps3.data.status.battery ){
  //     battery = Ps3.data.status.battery;
  //     Serial.print("The controller battery is ");
  //     if( battery == ps3_status_battery_charging )      Serial.println("charging");
  //     else if( battery == ps3_status_battery_full )     Serial.println("FULL");
  //     else if( battery == ps3_status_battery_high )     Serial.println("HIGH");
  //     else if( battery == ps3_status_battery_low)       Serial.println("LOW");
  //     else if( battery == ps3_status_battery_dying )    Serial.println("DYING");
  //     else if( battery == ps3_status_battery_shutdown ) Serial.println("SHUTDOWN");
  //     else Serial.println("UNDEFINED");
  // }
}

void onConnect() {
  String address = Ps3.getAddress();

  Serial.print("The ESP32's Bluetooth MAC address is: ");
  Serial.println(address);

  Ps3.setPlayer(0);

  servoBody.attach(PIN_BODY, 500, 2400);
  servoLeftArm.attach(PIN_LEFT_ARM, 500, 2400);
  servoRightArm.attach(PIN_RIGHT_ARM, 500, 2400);

  pinMode(PIN_LEFT_GUN, OUTPUT);
  pinMode(PIN_RIGHT_GUN, OUTPUT);

  pinMode(PIN_TRACK_A1, OUTPUT);
  pinMode(PIN_TRACK_A2, OUTPUT);
  pinMode(PIN_TRACK_B1, OUTPUT);
  pinMode(PIN_TRACK_B2, OUTPUT);

  init();
}

void setup()
{
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector

  Serial.begin(115200);
  Ps3.attach(notify);
  Ps3.attachOnConnect(onConnect);
  Ps3.begin("44:44:44:44:44:44");
}

void loop()
{
  if(!Ps3.isConnected())
    return;
}
