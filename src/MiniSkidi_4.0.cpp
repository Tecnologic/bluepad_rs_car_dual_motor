#include <Arduino.h>
#include <Bluepad32.h>
#include "Cdrv8833.h"

#define rightMotor0 25
#define rightMotor0Dir HIGH
#define rightMotor1 26
#define rightMotor1Dir HIGH

#define leftMotor0 33
#define leftMotor0Dir HIGH
#define leftMotor1 32
#define leftMotor1Dir HIGH

#define armMotor0 21
#define armMotor0Dir HIGH
#define armMotor1 19
#define armMotor1Dir HIGH

#define steeringServoPin 23
#define clawServoPin 22

#define auxLights0 16
#define auxLights1 17

#define batteryPin 34 // ADC pin for battery voltage measurement

constexpr int32_t steeringDriveMix = 25; // Steering drive mix factor, range: 0-100. 0 = no steering, 100 = full steering
constexpr int32_t stickDeadZone = 20;
constexpr int32_t triggerDeadZone = 20;
constexpr int32_t triggerMaxValue = 1023;


constexpr int32_t minBatteryVoltage = 6600; // Minimum battery voltage in millivolts (6,6V = 2S LiPo fully discharged)
constexpr int32_t warnBatteryVoltage = 7000; // Warning battery voltage in millivolts (7,0V = 2S LiPo low warning threshold)

constexpr bool leftMotorReversed = false; // Set to true if the left motor is reversed
constexpr bool rightMotorReversed = false; // Set to true if the right motor is reversed
constexpr bool armMotorReversed = false; // Set to true if the arm motor is reversed

constexpr bool torqueVectoringReversed = false; // Set to true if the torque vectoring is reversed
constexpr bool steeringServoReversed = true; // Set to true if the steering servo is reversed

Cdrv8833 rightMotor;
Cdrv8833 leftMotor;
Cdrv8833 armMotor;


constexpr int steeringServoMax = 1650; // Maximum pulse width for steering servo in microseconds
constexpr int steeringServoMin = 1350;  // Minimum pulse width for steering servo in microseconds

constexpr int clawServoMax = 2000;    // Maximum pulse width for claw servo in microseconds
constexpr int clawServoMin = 1000;   // Minimum pulse width for claw servo in microseconds


bool auxLightsOn = true;
bool turnLightsLeftOn = false;
bool turnLightsRightOn = false;


unsigned long lastWiggleTime = 0;
int wiggleCount = 0;
int wiggleDirection = 1;
unsigned long wiggleDelay = 100;
bool shouldWiggle = false;
bool yPressed = false;

ControllerPtr controller;

void steeringServoWrite(int value) {
  // Convert the value to a range suitable for the servo
  int servoValue = map(value, 0, 20000, 0, 65535); // 20000 is the max pulse width in microseconds for 50Hz PWM
  // Ensure the value is within the range of 0 to 65535 for 16-bit PWM
  servoValue = constrain(servoValue, 0, 65535);
  // Write the value to the LEDC channel for the steering servo
  ledcWrite(0, servoValue);
}

void clawServoWrite(int value) {
  // Convert the value to a range suitable for the servo
  int servoValue = map(value, 0, 20000, 0, 65535); // 20000 is the max pulse width in microseconds for 50Hz PWM
  // Ensure the value is within the range of 0 to 65535 for 16-bit PWM
  servoValue = constrain(servoValue, 0, 65535);
  // Write the value to the LEDC channel for the claw servo
  ledcWrite(1, servoValue);
}


// This callback gets called any time a new gamepad is connected.
void onConnectedController(ControllerPtr ctl) {
  if (controller == nullptr) {
    Serial.printf("CALLBACK: Controller is connected");
    ControllerProperties properties = ctl->getProperties();
    Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id, properties.product_id);
    controller = ctl;
    ctl->setColorLED(255, 0, 0);
    shouldWiggle = true;
    ctl->playDualRumble(0 /* delayedStartMs */, 250 /* durationMs */, 0x80 /* weakMagnitude */, 0x40 /* strongMagnitude */);

    // here the steering servo power is attached 
    armMotor.move(100);

  } else {
    Serial.println("CALLBACK: Controller connected, but could not found empty slot");
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  if (controller == ctl) {
    Serial.printf("CALLBACK: Controller disconnected");
    controller = nullptr;
    rightMotor.stop();
    leftMotor.stop();
    armMotor.stop();
    steeringServoWrite(steeringServoMax);
    clawServoWrite(clawServoMin);
    digitalWrite(auxLights0, LOW);
    digitalWrite(auxLights1, LOW);
  } else {
    Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
  }

}

void processGamepad(ControllerPtr ctl) {
  int32_t steeringValue = ctl->axisRX();
  int32_t throttleValue = ctl->axisY();
  int32_t forwardTriggerValue = ctl->throttle();
  int32_t reverseTriggerValue = ctl->brake();


  Serial.println("RX: " + String(steeringValue) + ", Y: " + String(throttleValue) + ", Throttle: " + String(forwardTriggerValue) + ", Brake: " + String(reverseTriggerValue));

  int32_t driveInput = 0;
  if (abs(throttleValue) > stickDeadZone) {
    driveInput = map(throttleValue, -512, 511, -100, 100);
  }

  if (forwardTriggerValue > triggerDeadZone || reverseTriggerValue > triggerDeadZone) {
    if (forwardTriggerValue >= reverseTriggerValue) {
      driveInput = map(forwardTriggerValue, triggerDeadZone, triggerMaxValue, 0, -100);
    } else {
      driveInput = map(reverseTriggerValue, triggerDeadZone, triggerMaxValue, 0, 100);
    }
  }

  int32_t steeringInput = 0;
  if (abs(steeringValue) > stickDeadZone) {
    steeringInput = map(steeringValue, -512, 511, -steeringDriveMix, steeringDriveMix);
  }

  if (torqueVectoringReversed) {
    steeringInput = -steeringInput;
  }

  if (abs(driveInput) < stickDeadZone) {
    steeringInput = 0;
  }


  steeringInput = (steeringInput * driveInput) / 100;

  int8_t leftMotorSpeed = constrain(driveInput - steeringInput, -100, 100);
  int8_t rightMotorSpeed = constrain(driveInput + steeringInput, -100, 100);

  int16_t steeringServoValue = (steeringServoMax + steeringServoMin) / 2;
  if (abs(steeringValue) > stickDeadZone) {

    if (steeringValue > 0) {
      turnLightsLeftOn = false;
      turnLightsRightOn = true;
    } else {
      turnLightsLeftOn = true;
      turnLightsRightOn = false;
    }

    steeringServoValue = map(steeringValue, -512, 511, steeringServoMin, steeringServoMax);
    if (steeringServoReversed) {
      steeringServoValue = map(steeringValue, -512, 511, steeringServoMax, steeringServoMin);
    }
  }

  if (leftMotorReversed) {
    leftMotorSpeed = -leftMotorSpeed;
  }
  if (rightMotorReversed) {
    rightMotorSpeed = -rightMotorSpeed;
  }

  if (driveInput == 0) {
    rightMotor.stop();
    leftMotor.stop();
  } else {
    leftMotor.move(leftMotorSpeed);
    rightMotor.move(rightMotorSpeed);
  }


  steeringServoWrite(steeringServoValue);

  if (ctl->a()) {
    shouldWiggle = true;
  }

  if (ctl->y() && !yPressed)
  {
    yPressed = true;

    if (!auxLightsOn)
    {
      auxLightsOn = true;
    }
    else
    {
      auxLightsOn = false;
    }
  } else if (!ctl->y() && yPressed) {
    yPressed = false;
  }
}

void processControllers() {
  if (controller && controller->isConnected() && controller->hasData()) {
    if (controller->isGamepad()) {
      processGamepad(controller);
    } else {
      Serial.println("Unsupported controller");
    }
  }
}

int32_t readBatteryVoltage()
{
  constexpr int32_t adcMaxValue = 4095;                           // ESP32 ADC max value for 12-bit resolution
  constexpr int32_t adcMaxVoltage = 3300 * (13000 + 4700) / 4700; // 3300 mV reference voltage, voltage divider with 12k and 4.7k resistors
  constexpr int32_t filterFactor = 64;                            // Filter factor for smoothing the battery voltage reading
  constexpr int32_t filterOrder = 2;                              // Number of filter stages (2 stages for smoothing)
  static int32_t filter[filterOrder + 1] = {0};                   // Array to store intermediate filter values

  // Read the battery voltage from the ADC pin
  int rawValue = analogRead(batteryPin);

  filter[0] = rawValue * adcMaxVoltage / adcMaxValue; // Convert ADC value to voltage in millivolts

  for (int32_t i = 1; i <= filterOrder; i++)
  {
    // Apply a multi-stage low-pass filter to smooth the battery voltage reading
    filter[i] = (filter[i] * (filterFactor - 1) + filter[i - 1]) / filterFactor;
  }

  return (filter[filterOrder]); // Return the smoothed battery voltage in millivolts
}

void setup() {

  Serial.begin(115200);
  Serial.setDebugOutput(true);

  Serial.println("MiniSkidi 4.0 starting...");
  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t* addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys();
  BP32.enableVirtualDevice(false);

  rightMotor.init(rightMotor0, rightMotor1, 5);
  rightMotor.setDecayMode(drv8833DecaySlow);
  rightMotor.swapDirection(false);
  leftMotor.init(leftMotor0, leftMotor1, 6);
  leftMotor.setDecayMode(drv8833DecaySlow);
  leftMotor.swapDirection(true);
  armMotor.init(armMotor0, armMotor1, 7);
  armMotor.setDecayMode(drv8833DecaySlow);

  rightMotor.stop();
  leftMotor.stop();
  armMotor.stop();


  pinMode(auxLights0, OUTPUT);
  pinMode(auxLights1, OUTPUT);
  
  digitalWrite(auxLights0, HIGH);
  digitalWrite(auxLights1, HIGH);

  pinMode(steeringServoPin, OUTPUT);
  pinMode(clawServoPin, OUTPUT);
  ledcSetup(0, 50, 16); // Set up PWM for servos
  ledcSetup(1, 50, 16); // Set up PWM for servos

  ledcAttachPin(steeringServoPin, 0);
  //ledcAttachPin(clawServoPin, 1);

  steeringServoWrite((steeringServoMax + steeringServoMin) / 2); // Center the steering servo

  pinMode(batteryPin, ANALOG); // Set up battery pin as analog input

  Serial.println("Ready.");
}

void wiggle() {
  unsigned long currentTime = millis();
  if (abs((int)(currentTime - lastWiggleTime)) >= wiggleDelay) {
    lastWiggleTime = currentTime;
    wiggleDirection = -wiggleDirection;
    wiggleCount++;
    rightMotor.move(wiggleDirection * 100);
    leftMotor.move(-1 * wiggleDirection * 100);
    if (wiggleCount >= 10) {
      rightMotor.brake();
      leftMotor.brake();
      wiggleCount = 0;
      shouldWiggle = false;
    }
  }
}

void loop() {
  unsigned long currentTime = millis();
  bool dataUpdated = BP32.update();
  if (dataUpdated) {
    processControllers();
  }
  if (shouldWiggle) {
    wiggle();
  }

  int32_t batteryVolts = readBatteryVoltage();

  if (currentTime % 1000 == 0) {
    Serial.printf("Battery voltage: %d mV\n", batteryVolts);
  }

  if (batteryVolts < warnBatteryVoltage)
  {
    // Blink aux lights to indicate low battery
    if (currentTime % 500 < 250)
    {
      digitalWrite(auxLights0, HIGH);
      digitalWrite(auxLights1, HIGH);
    }
    else
    {
      digitalWrite(auxLights0, LOW);
      digitalWrite(auxLights1, LOW);
    }

    if (batteryVolts < minBatteryVoltage)
    {
      if (currentTime % 500 == 0)
      {
        Serial.println("Battery voltage is low, stopping motors and servos.");
      }
      rightMotor.stop();
      leftMotor.stop();
      armMotor.stop();
    }
    else if (currentTime % 500 == 0)
    {
      Serial.printf("Warning: Battery voltage is low (%d mV), consider recharging.\n", batteryVolts);
    }
  }
  else
  {
    static bool lastAuxLightsOn = false;
    if (auxLightsOn != lastAuxLightsOn) {
      lastAuxLightsOn = auxLightsOn;
      Serial.printf("Aux lights turned %s\n", auxLightsOn ? "ON" : "OFF");
    }

    if (auxLightsOn) {
      digitalWrite(auxLights0, HIGH);
      digitalWrite(auxLights1, HIGH);
    } else {
      digitalWrite(auxLights0, LOW);
      digitalWrite(auxLights1, LOW);
    }

    if (currentTime % 500 == 0)
    {
      static bool lastTurnLightsLeftOn = false;
      static bool lastTurnLightsRightOn = false;

      if (turnLightsLeftOn != lastTurnLightsLeftOn) {
        digitalWrite(auxLights0, turnLightsLeftOn ? HIGH : LOW);
        lastTurnLightsLeftOn = turnLightsLeftOn;
      }
      if (turnLightsRightOn != lastTurnLightsRightOn) {
        digitalWrite(auxLights1, turnLightsRightOn ? HIGH : LOW);
        lastTurnLightsRightOn = turnLightsRightOn;
      }
    }
  }
}