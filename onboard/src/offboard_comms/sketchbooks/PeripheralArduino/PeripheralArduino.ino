// from L to R: data (pin 4), power (5V), ground
#include <Servo.h>
#include <Arduino.h>
#include <Wire.h>
#include "DHT11.h"
#include "MS5837.h"
#include "Robot.hpp"
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

unsigned long currentTime;
unsigned long prevTime = 0;


void setup() {
  switch (ROBOT_NAME) {
    case OOGWAY:
      robot = new Oogway(false, VOLTAGE_DELAY, PRESSURE_DELAY, TEMP_HUMIDITY_DELAY, SERVO_DELAY);
      break;
    case OOGWAY_SHELL:
      robot = new Oogway(true, VOLTAGE_DELAY, PRESSURE_DELAY, TEMP_HUMIDITY_DELAY, SERVO_DELAY);
      break;
    case CRUSH:
      // robot = new Oogway(false, VOLTAGE_DELAY, PRESSURE_DELAY, TEMP_HUMIDITY_DELAY, SERVO_DELAY);
      break;
    default:
      robot = new Oogway(false, VOLTAGE_DELAY, PRESSURE_DELAY, TEMP_HUMIDITY_DELAY, SERVO_DELAY);
  }
}

void loop() {
  currentTime = millis();

  // run all functions
  robot->process();
}