#include <Arduino.h>

#include <Ps3Controller.h>
#include <ESP32Servo.h>

#include "soc/soc.h"             // disable brownout problems
#include "soc/rtc_cntl_reg.h"    // disable brownout problems

#include "DFMiniMp3.h"

// These are all GPIO pins on the ESP32
// Recommended pins include 2,4,12-19,21-23,25-27,32-33
// for the ESP32-S2 the GPIO pins are 1-21,26,33-42
#define PIN_LEFT_ARM 22
#define PIN_RIGHT_ARM 25
#define PIN_BODY 27

#define PIN_LEFT_GUN 21
#define PIN_RIGHT_GUN 32

#define PIN_TRACK_A1 26
#define PIN_TRACK_A2 18
#define PIN_TRACK_B1 33
#define PIN_TRACK_B2 19

#define PIN_RX 16 // FIXED
#define PIN_TX 17 // FIXED

#define PIN_POWER 2

// #define PIN_CANNON 4

// FREE 23


#define CHANNEL_A1 12
#define CHANNEL_A2 13
#define CHANNEL_B1 14
#define CHANNEL_B2 15

#define MAX_VOLUME 18

Servo servoBody;
Servo servoLeftArm, servoRightArm;

class Mp3Notify;
typedef DFMiniMp3<HardwareSerial, Mp3Notify> DfMp3;
HardwareSerial mySerial(2); // 16, 17
DfMp3 dfmp3(mySerial);
int volume = MAX_VOLUME; // 0~30

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

  servoBody.attach(PIN_BODY, 500, 2400);
  servoLeftArm.attach(PIN_LEFT_ARM, 500, 2400);
  servoRightArm.attach(PIN_RIGHT_ARM, 500, 2400);

  servoBody.write(center);
  servoLeftArm.write(center);
  servoRightArm.write(center);

  pinMode(PIN_POWER, OUTPUT);
  digitalWrite(PIN_POWER, HIGH);

  // pinMode(PIN_CANNON, OUTPUT);

  dfmp3.playMp3FolderTrack(3);
}

void fire(int pin) {
  for (int i = 0; i < 4; i ++) {
    digitalWrite(pin, HIGH);
    delay(80);
    digitalWrite(pin, LOW);
    delay(80);
  }
}

int battery = 0;
void notify()
{
  // RESET
  if (Ps3.event.button_down.start) {
    init();
  }

  // Gatling
  if ((Ps3.event.button_down.cross) || (Ps3.event.button_down.circle)) {
    dfmp3.playMp3FolderTrack(1);
  }
  if (Ps3.event.button_down.cross) {
    fire(PIN_LEFT_GUN);
  }
  if (Ps3.event.button_down.circle) {
    fire(PIN_RIGHT_GUN);
  }

  // Cannon
  if ((Ps3.event.button_down.square) || (Ps3.event.button_down.triangle)) {
    dfmp3.playMp3FolderTrack(2);

    // digitalWrite(PIN_CANNON, HIGH);

    // Back
    ledcWrite(CHANNEL_B1, 0);
    ledcWrite(CHANNEL_B2, 255);

    ledcWrite(CHANNEL_A1, 0);
    ledcWrite(CHANNEL_A2, 255);

    delay(100);

    ledcWrite(CHANNEL_B1, 0);
    ledcWrite(CHANNEL_B2, 0);

    ledcWrite(CHANNEL_A1, 0);
    ledcWrite(CHANNEL_A2, 0);

    // delay(300);
    // digitalWrite(PIN_CANNON, LOW);
  }


  if (Ps3.event.button_down.l1) {
    Serial.println("L1");
    leftArmAngle = max(leftArmAngle - angleStep, center - 45);
    servoLeftArm.write(leftArmAngle);
  }
  if (Ps3.event.button_down.l2) {
    Serial.println("L2");
    leftArmAngle = min(leftArmAngle + angleStep, center + 45);
    servoLeftArm.write(leftArmAngle);
  }
  if (Ps3.event.button_down.r1) {
    Serial.println("R1");
    rightArmAngle = min(rightArmAngle + angleStep, center + 45);
    servoRightArm.write(rightArmAngle);
  }
  if (Ps3.event.button_down.r2) {
    Serial.println("R2");
    rightArmAngle = max(rightArmAngle - angleStep, center - 45);
    servoRightArm.write(rightArmAngle);
  }

  // Body
  if (Ps3.event.button_down.left) {
    Serial.println("Left");
    bodyAngle = min(bodyAngle + 5, 180);
    servoBody.write(bodyAngle);
  }
  if (Ps3.event.button_down.right) {
    Serial.println("Right");
    bodyAngle = max(bodyAngle - 5, 0);
    servoBody.write(bodyAngle);
  }

  // Volume
  if (Ps3.event.analog_changed.button.up) {
    volume = min(volume + 2, MAX_VOLUME);
    dfmp3.setVolume(volume);
  }
  if (Ps3.event.analog_changed.button.down) {
    volume = max(volume - 2, 0);
    dfmp3.setVolume(volume);
  }

  // Track
  int absLy = abs(Ps3.event.analog_changed.stick.ly);
  if (absLy < 10) {
    ledcWrite(CHANNEL_B1, 0);
    ledcWrite(CHANNEL_B2, 0);
  } else {
    if (Ps3.event.analog_changed.stick.ly < -10) {
      ledcWrite(CHANNEL_B1, absLy * 4);
      ledcWrite(CHANNEL_B2, 0);
    }
    else if (Ps3.event.analog_changed.stick.ly > 10) {
      ledcWrite(CHANNEL_B1, 0);
      ledcWrite(CHANNEL_B2, absLy * 4);
    }
  }

  int absRy = abs(Ps3.event.analog_changed.stick.ry);
  if (absRy < 10) {
    ledcWrite(CHANNEL_A1, 0);
    ledcWrite(CHANNEL_A2, 0);    
  } else {
    if (Ps3.event.analog_changed.stick.ry < -10) {
      ledcWrite(CHANNEL_A1, absRy * 4);
      ledcWrite(CHANNEL_A2, 0);
    }
    else if (Ps3.event.analog_changed.stick.ry > 10) {
      ledcWrite(CHANNEL_A1, 0);
      ledcWrite(CHANNEL_A2, absRy * 4);
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
  Serial.println("Connected");

  String address = Ps3.getAddress();

  Serial.print("The ESP32's Bluetooth MAC address is: ");
  Serial.println(address);

  init();

  servoBody.attach(PIN_BODY, 500, 2400);
  servoLeftArm.attach(PIN_LEFT_ARM, 500, 2400);
  servoRightArm.attach(PIN_RIGHT_ARM, 500, 2400);

  pinMode(PIN_LEFT_GUN, OUTPUT);
  pinMode(PIN_RIGHT_GUN, OUTPUT);


  Serial.printf("Setup CHANNEL_A1 %d\n", CHANNEL_A1);
  ledcSetup(CHANNEL_A1, 5000, 7); // 0~127
  Serial.printf("Setup CHANNEL_A2 %d\n", CHANNEL_A2);
  ledcSetup(CHANNEL_A2, 5000, 7); // 0~127
  Serial.printf("Setup CHANNEL_B1 %d\n", CHANNEL_B1);
  ledcSetup(CHANNEL_B1, 5000, 7); // 0~127
  Serial.printf("Setup CHANNEL_B2 %d\n", CHANNEL_B2);
  ledcSetup(CHANNEL_B2, 5000, 7); // 0~127

  Serial.printf("Attach CHANNEL_A1 %d\n", PIN_TRACK_A1);
  ledcAttachPin(PIN_TRACK_A1, CHANNEL_A1);
  Serial.printf("Attach CHANNEL_A2 %d\n", PIN_TRACK_A2);
  ledcAttachPin(PIN_TRACK_A2, CHANNEL_A2);
  Serial.printf("Attach CHANNEL_B1 %d\n", PIN_TRACK_B1);
  ledcAttachPin(PIN_TRACK_B1, CHANNEL_B1);
  Serial.printf("Attach CHANNEL_B2 %d\n", PIN_TRACK_B2);
  ledcAttachPin(PIN_TRACK_B2, CHANNEL_B2);

  init();

  dfmp3.playMp3FolderTrack(3);
}

void onDisconnect() {
  Serial.println("Disconnected");
  Ps3.end();
  initPs3();
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
  Serial.begin(115200);
  init();
  initPs3();

  dfmp3.begin(9600, 1000);
  dfmp3.reset();

  while(!dfmp3.isOnline()) {
    delay(10);
  }

  dfmp3.setVolume(volume);
}

void loop() {
  if(!Ps3.isConnected())
    return;

  dfmp3.loop();
  delay(1);
}

//----------------------------------------------------------------------------------
class Mp3Notify
{
public:
  static void PrintlnSourceAction(DfMp3_PlaySources source, const char *action)
  {
#ifdef _DEBUG
    if (source & DfMp3_PlaySources_Sd)
    {
      Serial.print("SD Card, ");
    }
    if (source & DfMp3_PlaySources_Usb)
    {
      Serial.print("USB Disk, ");
    }
    if (source & DfMp3_PlaySources_Flash)
    {
      Serial.print("Flash, ");
    }
    Serial.println(action);
#endif
  }
  static void OnError(DfMp3 &mp3, uint16_t errorCode)
  {
#ifdef _DEBUG
    // see DfMp3_Error for code meaning
    Serial.println();
    Serial.print("Com Error ");
    switch (errorCode)
    {
    case DfMp3_Error_Busy:
      Serial.println("Busy");
      break;
    case DfMp3_Error_Sleeping:
      Serial.println("Sleeping");
      break;
    case DfMp3_Error_SerialWrongStack:
      Serial.println("Serial Wrong Stack");
      break;

    case DfMp3_Error_RxTimeout:
      Serial.println("Rx Timeout!!!");
      break;
    case DfMp3_Error_PacketSize:
      Serial.println("Wrong Packet Size!!!");
      break;
    case DfMp3_Error_PacketHeader:
      Serial.println("Wrong Packet Header!!!");
      break;
    case DfMp3_Error_PacketChecksum:
      Serial.println("Wrong Packet Checksum!!!");
      break;

    default:
      Serial.println(errorCode, HEX);
      break;
    }
#endif
  }
  static void OnPlayFinished(DfMp3 &mp3, DfMp3_PlaySources source, uint16_t track)
  {
#ifdef _DEBUG
    Serial.print("Play finished for #");
    Serial.println(track);
#endif
  }
  static void OnPlaySourceOnline(DfMp3 &mp3, DfMp3_PlaySources source)
  {
    PrintlnSourceAction(source, "online");
  }
  static void OnPlaySourceInserted(DfMp3 &mp3, DfMp3_PlaySources source)
  {
    PrintlnSourceAction(source, "inserted");
  }
  static void OnPlaySourceRemoved(DfMp3 &mp3, DfMp3_PlaySources source)
  {
    PrintlnSourceAction(source, "removed");
  }
};