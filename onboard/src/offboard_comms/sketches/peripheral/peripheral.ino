// from L to R: data (pin 4), power (5V), ground
#include <Servo.h>
#include <Arduino.h>
#include <Wire.h>
#include "DHT11.h"
#include "MS5837.h"
#include "Oogway.cpp"
#include "Crush.cpp"

#define OOGWAY 0
#define OOGWAY_SHELL 1
#define CRUSH 2

#define VOLTAGE_DELAY 100
#define PRESSURE_DELAY 50
#define TEMP_HUMIDITY_DELAY 1000
#define SERVO_DELAY 50

// Baud rate for serial communication with Blue Robotics Bar30 High-Resolution 300m Depth/Pressure Sensor
#define BAUD_RATE 9600

Robot* robot;
bool valid_robot = true;

void setupGyroTrigger() {
    // Set OC2A (Pin 11) and OC2B (Pin 3) as outputs
    pinMode(11, OUTPUT); // OC2A
    pinMode(3, OUTPUT);  // OC2B

    // Stop Timer2
    TCCR2A = 0;
    TCCR2B = 0;
    TCNT2 = 0;

    // Set CTC mode (Clear Timer on Compare Match)
    TCCR2A = (1 << COM2A0) | (1 << COM2B0) | (1 << WGM21); // Toggle OC2A and OC2B on match, CTC mode
    TCCR2B = (1 << CS22); // Prescaler = 64

    OCR2A = 124;
}

void setup() {
  Serial.begin(BAUD_RATE);

  switch (ROBOT_NAME) {
    case OOGWAY:
      robot = new Oogway(VOLTAGE_DELAY, PRESSURE_DELAY, TEMP_HUMIDITY_DELAY, SERVO_DELAY, false);
      setupGyroTrigger();
      break;
    case OOGWAY_SHELL:
      robot = new Oogway(VOLTAGE_DELAY, PRESSURE_DELAY, TEMP_HUMIDITY_DELAY, SERVO_DELAY, true);
      break;
    case CRUSH:
      robot = new Crush(VOLTAGE_DELAY, PRESSURE_DELAY, TEMP_HUMIDITY_DELAY, SERVO_DELAY, false);
      break;
    default:
      valid_robot = false;
  }
}

void loop() {
  if (valid_robot) {
    // Call all sensors and process servo commands
    robot->process();
  } else {
    Serial.println("Error: Invalid ROBOT_NAME: " + String(ROBOT_NAME));
    delay(500); // Delay to avoid flooding the serial output
  }
}