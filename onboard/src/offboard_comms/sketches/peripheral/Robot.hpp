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
        // Delay between each sensor reading
        int voltageDelay;
        int pressureDelay;
        int tempHumidityDelay;
        int servoDelay;

        // Current time in milliseconds
        int currentTime;

        // Last time each sensor was called
        int prevTimeVoltage;
        int prevTimePressure;
        int prevTimeTempHumidity;
        int prevTimeServo;

        // Lists to store each sensor, and size of each list
        Voltage* voltageList[MAX_VOLTAGE_SENSORS];
        Pressure* pressureList[MAX_PRESSURE_SENSORS];
        TempHumidity* tempHumidityList[MAX_TEMP_HUMIDITY_SENSORS];
        RobotServo* servoList[MAX_SERVOS];
        int numVoltage, numPressure, numTempHumidity, numServos;

    public:
        Robot(int voltageDelay, int pressureDelay, int tempHumidityDelay, int servoDelay)
            : voltageDelay(voltageDelay), pressureDelay(pressureDelay), tempHumidityDelay(tempHumidityDelay), servoDelay(servoDelay) {
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

        // Process method to be called in the main loop
        // This method will call the methods to print each sensor's data
        // It will also check for incoming serial data to control the servos
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
                // Check for incoming serial data to control servos
                if (Serial.available() > 0) {
                    String input = Serial.readString();
                    int colonIndex = input.indexOf(":");

                    // Make sure colon exists in input
                    if (colonIndex != -1) {
                        String tag = input.substring(0, colonIndex);
                        int pwm = input.substring(colonIndex + 1).toInt(); // If the string after the colon is not a number, it will return 0
                        for (int i = 0; i < numServos; ++i) {
                            if (servoList[i]->getTag() == tag) {
                                servoList[i]->callServo(pwm);
                                break;
                            }
                        }
                    }
                }
            }
        }
};