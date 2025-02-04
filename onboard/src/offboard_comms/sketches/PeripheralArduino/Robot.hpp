#include "Voltage.hpp"
#include "Pressure.hpp"
#include "TempHumidity.hpp"
#include "RobotServo.hpp"
#include <Arduino.h>

#define MAX_VOLTAGE_SENSORS 10
#define MAX_PRESSURE_SENSORS 10
#define MAX_TEMP_HUMIDITY_SENSORS 10
#define MAX_SERVOS 10

class Robot {
    protected:
        bool isShell;
        int voltageDelay;
        int pressureDelay;
        int tempHumidityDelay;
        int servoDelay;

        int currentTime;
        int prevTimeVoltage;
        int prevTimePressure;
        int prevTimeTempHumidity;
        int prevTimeServo;

        Voltage* voltageList[MAX_VOLTAGE_SENSORS];
        Pressure* pressureList[MAX_PRESSURE_SENSORS];
        TempHumidity* tempHumidityList[MAX_TEMP_HUMIDITY_SENSORS];
        RobotServo* servoList[MAX_SERVOS];
        int numVoltage, numPressure, numTempHumidity, numServos;

    public:
        Robot(int voltageDelay, int pressureDelay, int tempHumidityDelay, int servoDelay) :
            voltageDelay(voltageDelay), pressureDelay(pressureDelay), tempHumidityDelay(tempHumidityDelay), servoDelay(servoDelay), isShell(false) {
            init();
        }

        void init() {
            prevTimeVoltage = 0;
            prevTimePressure = 0;
            prevTimeTempHumidity = 0;
            prevTimeServo = 0;
            numVoltage = 0;
            numPressure = 0;
            numTempHumidity = 0;
            numServos = 0;
        }

        void addVoltageSensor(Voltage* v) {
            if (numVoltage < MAX_VOLTAGE_SENSORS) {
                voltageList[numVoltage++] = v;
            }
        }

        void addPressureSensor(Pressure* p) {
            if (numPressure < MAX_PRESSURE_SENSORS) {
                pressureList[numPressure++] = p;
            }
        }

        void addTempHumiditySensor(TempHumidity* th) {
            if (numTempHumidity < MAX_TEMP_HUMIDITY_SENSORS) {
                tempHumidityList[numTempHumidity++] = th;
            }
        }

        void addServo(RobotServo* s) {
            if (numServos < MAX_SERVOS) {
                servoList[numServos++] = s;
            }
        }

        void process() {
            currentTime = millis();

            if (currentTime - prevTimeVoltage >= voltageDelay) {
                prevTimeVoltage = currentTime;
                for (int i = 0; i < numVoltage; ++i) {
                    voltageList[i]->callVoltage();
                }
            }

            if (currentTime - prevTimePressure >= pressureDelay) {
                prevTimePressure = currentTime;
                for (int i = 0; i < numPressure; ++i) {
                    pressureList[i]->callPressure();
                }
            }

            if (currentTime - prevTimeTempHumidity >= tempHumidityDelay) {
                prevTimeTempHumidity = currentTime;
                for (int i = 0; i < numTempHumidity; ++i) {
                    tempHumidityList[i]->callTempHumidity();
                }
            }

            // Continuously update servos to check if they need to return to stopPWM
            for (int i = 0; i < numServos; ++i) {
                servoList[i]->updateServo();
            }

            if (currentTime - prevTimeServo >= servoDelay) {
                prevTimeServo = currentTime;
                if (Serial.available() > 0) {
                    String input = Serial.readString();
                    int colonIndex = input.indexOf(":");
                    String id = input.substring(0, colonIndex);
                    int pwm = input.substring(colonIndex + 1).toInt();

                    for (int i = 0; i < numServos; ++i) {
                        if (servoList[i]->getTag() == id) {
                            servoList[i]->callServo(pwm);
                            break;
                        }
                    }
                }
            }
        }
};