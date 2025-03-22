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

#define GYRO_PIN_A 5
#define GYRO_PIN_B 6

// Baud rate for serial communication with Blue Robotics Bar30 High-Resolution 300m Depth/Pressure Sensor
#define BAUD_RATE 9600

Robot* robot;
bool valid_robot = true;

void setupGyroTrigger() {
  pinMode(GYRO_PIN_A, OUTPUT);
  pinMode(GYRO_PIN_B, OUTPUT);

  noInterrupts(); // Disable interrupts during timer setup

  // Timer2: CTC mode
  TCCR2A = 0;
  TCCR2B = 0;

  TCCR2A |= (1 << WGM21);  // CTC mode

  // Prescaler: 64 → 16MHz / 64 = 250kHz (4 µs per tick)
  // 500 µs / 4 µs = 125 ticks → OCR2A = 124
  OCR2A = 124;

  TCCR2B |= (1 << CS22);  // Prescaler 64
  TIMSK2 |= (1 << OCIE2A); // Enable Timer2 compare match interrupt

  interrupts(); // Enable interrupts
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

ISR(TIMER2_COMPA_vect) {
  static bool state = false;
  state = !state;

  digitalWrite(GYRO_PIN_A, state);
  digitalWrite(GYRO_PIN_B, !state); // Inverted
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