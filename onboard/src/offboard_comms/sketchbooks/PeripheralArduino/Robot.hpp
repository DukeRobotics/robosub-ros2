#include "Voltage.hpp"
#include "Pressure.hpp"
#include "TempHumidity.hpp"
#include "RobotServo.hpp"
#include <Arduino.h>

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

        std::vector<Voltage> voltageList;
        std::vector<Pressure> pressureList;
        std::vector<TempHumidity> tempHumidityList;
        std::map<String, RobotServo> servoMap;

    public:
        Robot(int voltageDelay, int pressureDelay, int tempHumidityDelay, int servoDelay) :
        voltageDelay(voltageDelay), pressureDelay(pressureDelay), tempHumidityDelay(tempHumidityDelay), servoDelay(servoDelay), isShell(isShell) {
            init();
        }

        void init() {
            prevTimeVoltage = 0;
            prevTimePressure = 0;
            prevTimeTempHumidity = 0;
            prevTimeServo = 0;
        }

        void process() {
            currentTime = millis();

            if (currentTime - prevTimeVoltage >= voltageDelay) {
                prevTimeVoltage = currentTime;

                for (Voltage v: this->voltageList) {
                    v.callVoltage();
                }
            }

            if (currentTime - prevTimePressure >= pressureDelay) {
                prevTimePressure = currentTime;

                for (Pressure p: this->pressureList) {
                    p.callPressure();
                }
            }

            if (currentTime - prevTimeTempHumidity >= tempHumidityDelay) {
                prevTimeTempHumidity = currentTime;

                for (TempHumidity th: this->tempHumidityList) {
                    th.callTempHumidity();
                }
            }

            if (currentTime - prevTimeServo >= servoDelay) {
                prevTimeServo = currentTime;

                if (Serial.available() > 0) {
                    String input = Serial.readString();
                    String id = input.substring(0,1);
                    int data = (int) input.substring(2);

                    if (this->servoMap.count(id)) {
                        RobotServo s = this->servoMap.at(id);
                        if ((s.getMinPWM() < data) && (data < s.getMaxPWM()))
                            s.callServo(data);
                    }
                }
            }
        }
};