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
  // Set up Timer2 to generate two complementary 1000Hz square waves on OC2A (pin D11) and OC2B (pin D3)
  // When OC2A is high, OC2B is low, and when OC2A is low, OC2B is high
  //
  // See https://www.gammon.com.au/images/Arduino/Timer_2.png for a diagram of how to configure Timer2
  // See https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf for the official documentation
  //
  // Hardware timers are used to generate precise timing signals, such as PWM signals, or in this case a trigger signal for the gyro.
  //
  // The ATmega328P has three hardware timers: Timer0, Timer1, and Timer2.
  // Timer0 is used for the Arduino's millis() and delay() functions, and Timer1 is used for the Servo library. Thus, Timer2 is available for use in this sketch.
  //

  // TCCR2A and TCCR2A are the control registers for Timer2; they define how the timer behaves
  // Set the registers to 0 to clear any previous settings
  TCCR2A = 0;
  TCCR2A = 0;

  // TCNT2 is the timer counter register; its value is incremented by the timer
  TCNT2 = 0;

  // Setting the WGM21 bit means the timer is set to CTC (Clear Timer on Compare Match) mode
  // This means that the timer will count up from 0 to the value in OCR2A, then reset to 0. OCR2B must be less than or equal to OCR2A.
  // Setting the COM2A0 and COM2B0 bits means the outputs of pins OC2A and OC2B will be toggled when the timer reaches OCR2A and OCR2B respectively
  TCCR2A = (1 << COM2A0) | (1 << COM2B0) | (1 << WGM21);

  // Setting the CS22 bit sets the prescale to 64, which means for every 64 clock cycles, the timer will increment by 1
  // The FOC2B bit is set to immediately toggle the OC2B output when the timer is started, ensuring that its value is opposite to OC2A and thus creating a complementary output
  TCCR2B = (1 << CS22) | (1 << FOC2B);

  // The OCR2A and OCR2B registers define the values at which the timer will toggle the outputs
  // The ATmega328P has a CPU clock speed of 16MHz, and with a prescaler of 64, the timer increments 16,000,000 / 64 = 250,000 times per second
  // To create a 1000Hz square wave with 50% duty cycle, the timer must toggle the outputs at 2000Hz
  // So set the OCR2A and OCR2B registers to 124, which means the timer will toggle the outputs 250,000 / 125 = 2000 times per second (the timer counts from 0 to 124, which is 125 counts)
  OCR2A = 124;
  OCR2B = 124;
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