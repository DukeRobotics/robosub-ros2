#include "Adafruit_PWMServoDriver.h"
#include "MultiplexedBasicESC.h"
#include <Arduino.h>

#define OOGWAY 0
#define OOGWAY_SHELL 1
#define CRUSH 2

#define BAUD_RATE 57600
#define THRUSTER_TIMEOUT_MS 1000
#define THRUSTER_STOP_PWM 1500
#define THRUSTER_PWM_MIN 1100
#define THRUSTER_PWM_MAX 1900

Adafruit_PWMServoDriver pwm_multiplexer(0x40);

int NUM_THRUSTERS;
int THRUSTER_PWM_OFFSET; // Hardware specific offset for PWMs -- refers to the robot-specific offsets

byte START_FLAG[] = {0xFF, 0xFF};

uint64_t last_cmd_ms_ts;

uint16_t* pwms;

MultiplexedBasicESC* thrusters;

bool startFlagDetected = false;

void write_pwms() {
    for (uint8_t i = 0; i < NUM_THRUSTERS; i++) {
        if (pwms[i] < THRUSTER_PWM_MIN || pwms[i] > THRUSTER_PWM_MAX) {
            return; // If any PWM value is out of range, return and don't write any PWMs
        }
    }

    for (uint8_t i = 0; i < NUM_THRUSTERS; i++) {
        thrusters[i].write(pwms[i] + THRUSTER_PWM_OFFSET);
    }
    last_cmd_ms_ts = millis();
}

void setup() {
    switch (ROBOT_NAME) {
        case OOGWAY:
            NUM_THRUSTERS = 8;
            THRUSTER_PWM_OFFSET = 0;
            break;
        case OOGWAY_SHELL:
            NUM_THRUSTERS = 8;
            THRUSTER_PWM_OFFSET = 57;
            break;
        case CRUSH:
            NUM_THRUSTERS = 6;
            THRUSTER_PWM_OFFSET = 0;
            break;
        default:
            NUM_THRUSTERS = 8;
            THRUSTER_PWM_OFFSET = 0;
    }

    pwms = new uint16_t[NUM_THRUSTERS];
    thrusters = new MultiplexedBasicESC[NUM_THRUSTERS];

    pwm_multiplexer.begin();
    for (uint8_t i = 0; i < NUM_THRUSTERS; i++) {
        thrusters[i].initialize(&pwm_multiplexer);
        thrusters[i].attach(i);
    }

    // Initialize the PWMs to stop
    for (uint8_t i = 0; i < NUM_THRUSTERS; i++) {
        pwms[i] = THRUSTER_STOP_PWM;
    }

    // Write the stop PWM to all thrusters to initialize them (proper beep sequence)
    write_pwms();

    Serial.begin(BAUD_RATE);
}

void loop() {
    // If we haven't received a new command in a while, stop all thrusters
    if (millis() - last_cmd_ms_ts > THRUSTER_TIMEOUT_MS) {
        for (uint8_t i = 0; i < NUM_THRUSTERS; i++) {
            pwms[i] = THRUSTER_STOP_PWM;
        }
        write_pwms();
    }

    // Make sure we have enough data to read the start flag and all the PWM values
    if (Serial.available() > 2 * NUM_THRUSTERS + 2) {

        // Read bytes from serial until we find the start flag, until we run out of serial data, or until we read 2 * NUM_THRUSTERS + 2 bytes
        byte window[2] = {0x00, 0x00};
        size_t bytesRead = 0;
        while (Serial.available() > 0 && bytesRead < (2 * NUM_THRUSTERS + 2) && !startFlagDetected) {
            window[0] = window[1];
            window[1] = Serial.read();
            if (window[0] == START_FLAG[0] && window[1] == START_FLAG[1]) {
                startFlagDetected = true;
            }
            bytesRead++;
        }

        // If the start flag is not detected, return to the start of loop() to read the next set of data or stop the thrusters
        if (!startFlagDetected) {
            return;
        }

        // Reset the flag for the next iteration
        startFlagDetected = false;

        // The start flag was detected, so read the PWM values for the thrusters, which should be NUM_THRUSTERS * 2 bytes
        byte incomingData[NUM_THRUSTERS * 2];
        bytesRead = 0;
        while (Serial.available() > 0 && bytesRead < sizeof(incomingData)) {
            incomingData[bytesRead] = Serial.read();
            bytesRead++;

            // If start flag is detected in the incoming data, it means the data doesn't have enough PWM values for all thrusters
            // So, return to the start of loop(), skip the previous while loop (since startFlagDetected = true), and read the PWM values
            if (bytesRead >= 2 && incomingData[bytesRead - 2] == START_FLAG[0] && incomingData[bytesRead - 1] == START_FLAG[1]) {
                startFlagDetected = true;
                return;
            }
        }

        // If we read the correct number of bytes, unpack the data into the pwms array (big-endian)
        if (bytesRead == sizeof(incomingData)) {
            for (size_t i = 0; i < NUM_THRUSTERS; i++) {
                pwms[i] = incomingData[2 * i + 1] | (incomingData[2 * i] << 8);
            }
            write_pwms();
        }
    }
}