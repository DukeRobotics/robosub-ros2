#ifndef ACTUATORS_HPP
#define ACTUATORS_HPP

#include <Arduino.h>
#include "Adafruit_PWMServoDriver.h"
#include "MultiplexedBasicESC.hpp"
#include "RobotServo.hpp"

#define OOGWAY 0
#define OOGWAY_SHELL 1
#define CRUSH 2

#ifndef ROBOT_NAME
#error "ROBOT_NAME is not defined. Please define ROBOT_NAME as OOGWAY, OOGWAY_SHELL, or CRUSH."
#endif

#if ROBOT_NAME == OOGWAY
constexpr uint16_t SERVO_MARKER = 8;
constexpr uint16_t SERVO_TORPEDO = 9;
constexpr ServoConfig SERVO_CONFIGS[] = {
    {SERVO_MARKER, 500, 1500, 2500, 'M', 1000},
    {SERVO_TORPEDO, 700, 1500, 2300, 'T', 1000},    
};
constexpr uint8_t NUM_THRUSTERS = 8;
constexpr uint8_t NUM_SERVOS = 2;
constexpr int16_t THRUSTER_PWM_OFFSET = 0;

#elif ROBOT_NAME == OOGWAY_SHELL
constexpr ServoConfig* SERVO_CONFIGS = nullptr;
constexpr uint8_t NUM_THRUSTERS = 8;
constexpr uint8_t NUM_SERVOS = 0;
constexpr int16_t THRUSTER_PWM_OFFSET = 57;

#elif ROBOT_NAME == CRUSH
constexpr uint16_t SERVO_MARKER = 5;
constexpr ServoConfig SERVO_CONFIGS[] = {
    {SERVO_MARKER, 1300, 1500, 1700, 'M', 1000},
};
constexpr uint8_t NUM_THRUSTERS = 8;
constexpr uint8_t NUM_SERVOS = 1;
constexpr int16_t THRUSTER_PWM_OFFSET = 210;

#else
#error "Invalid ROBOT_NAME. Please define ROBOT_NAME as OOGWAY, OOGWAY_SHELL, or CRUSH."
#endif


constexpr uint8_t MAX_SERVOS = 5;
constexpr uint16_t THRUSTER_PWM_MIN = 1100;
constexpr uint16_t THRUSTER_PWM_MAX = 1900;
constexpr uint16_t THRUSTER_STOP_PWM = 1500;
constexpr uint32_t THRUSTER_TIMEOUT_MS = 1000;




class Actuators {
    Adafruit_PWMServoDriver pwm_multiplexer;
    uint16_t pwmBuffer[NUM_THRUSTERS];
    MultiplexedBasicESC thrusters[NUM_THRUSTERS];
    RobotServo servos[MAX_SERVOS];
    uint32_t last_thruster_cmd_ms_ts;
    //uint32_t last_servo_cmd_ms_ts;
public:
    Actuators() : pwm_multiplexer(0x40) {}
    void setup(){   

        pwm_multiplexer.begin();

        for (uint8_t i = 0; i < NUM_THRUSTERS; i++) {
            thrusters[i].initialize(&pwm_multiplexer);
            thrusters[i].attach(i);
        }

        // Initialize the PWMs to stop
        for (uint8_t i = 0; i < NUM_THRUSTERS; i++) {
            pwmBuffer[i] = THRUSTER_STOP_PWM;
        }

        // Write the stop PWM to all thrusters to initialize them (proper beep sequence)
        applyThrusterPwms(pwmBuffer);

        for (uint8_t i = 0; i < NUM_SERVOS; i++) {
            servos[i] = RobotServo(SERVO_CONFIGS[i].pinNum, SERVO_CONFIGS[i].minPWM, SERVO_CONFIGS[i].stopPWM, SERVO_CONFIGS[i].maxPWM, SERVO_CONFIGS[i].tag, SERVO_CONFIGS[i].delay);
        }

        for (uint8_t i = 0; i < NUM_SERVOS; i++) {
            servos[i].begin();
        }

    }

    void applyThrusterPwms(const uint16_t* pwms) {
        for (uint8_t i = 0; i < NUM_THRUSTERS; i++) {
            if (pwms[i] < THRUSTER_PWM_MIN || pwms[i] > THRUSTER_PWM_MAX) {
                return; // If any PWM value is out of range, return and don't write any PWMs
            }
        }

        for (uint8_t i = 0; i < NUM_THRUSTERS; i++) {
            thrusters[i].write(pwms[i] + THRUSTER_PWM_OFFSET);
        }

        last_thruster_cmd_ms_ts = millis();
    }

    void applyServoPwms(const uint16_t servoID, const uint16_t pwm) {
        for(int i = 0; i < NUM_SERVOS; i++) {
            if(servos[i].getTag() == servoID) {
                servos[i].callServo(pwm);
                break;
            }
        }

        //last_servo_cmd_ms_ts = millis();
    }

    void stopThrusters() {
        for (uint8_t i = 0; i < NUM_THRUSTERS; i++) {
            pwmBuffer[i] = THRUSTER_STOP_PWM;
        }
        applyThrusterPwms(pwmBuffer);
    }
    
    void stopServos() {
        for(int i = 0; i < NUM_SERVOS; i++) {
            servos[i].stopServo();
        }
    }

    void stopAll() {
        stopThrusters();
        stopServos();
    }

    void updateServos() {
        for(int i = 0; i < NUM_SERVOS; i++) {
            servos[i].updateServo();
        }
    }

    void updateThrusters() {
        if (millis() - last_thruster_cmd_ms_ts > THRUSTER_TIMEOUT_MS) {
            stopThrusters();
        }
    }

    void updateAll(){
        updateServos();
        updateThrusters();
    }

};

#endif