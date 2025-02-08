// from L to R: data (pin 4), power (5V), ground
#include <Servo.h>
#include <Arduino.h>
#include <Wire.h>
#include "DHT11.h"
#include "MS5837.h"
#include "Oogway.cpp"

#define OOGWAY 0
#define OOGWAY_SHELL 1
#define CRUSH 2

#define VOLTAGE_DELAY 100
#define PRESSURE_DELAY 0
#define TEMP_HUMIDITY_DELAY 1000
#define SERVO_DELAY 0

// Baud rate for serial communication with Blue Robotics Bar30 High-Resolution 300m Depth/Pressure Sensor
#define BAUD_RATE 9600

Robot* robot;
bool valid_robot = true;

void setup() {
  Serial.begin(BAUD_RATE);

  switch (ROBOT_NAME) {
    case OOGWAY:
      robot = new Oogway(VOLTAGE_DELAY, PRESSURE_DELAY, TEMP_HUMIDITY_DELAY, SERVO_DELAY, false);
      break;
    case OOGWAY_SHELL:
      robot = new Oogway(VOLTAGE_DELAY, PRESSURE_DELAY, TEMP_HUMIDITY_DELAY, SERVO_DELAY, true);
      break;
    case CRUSH:
      // TODO: crush
      break;
    default:
      valid_robot = false;
  }
}

void loop() {
  if (valid_robot) {
    // Run all functions
    robot->process();
  } else {
    // Continuously print error message
    Serial.println("Error: Invalid ROBOT_NAME: " + String(ROBOT_NAME));
    delay(500); // Delay to avoid flooding the serial output
  }
}