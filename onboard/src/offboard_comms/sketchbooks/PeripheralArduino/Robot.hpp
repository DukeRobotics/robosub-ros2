#include "Voltage.hpp"
#include "Pressure.hpp"
#include "TempHumidity.hpp"
#include "Servo.hpp"
#include <Arduino.h>
#include <map>
#include <string>
#include <vector>

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
        std::map<std::string, Servo> servoMap;

    public:
        Robot(bool isShell = false, int voltageDelay, int pressureDelay, int tempHumidityDelay, int servoDelay) :
        isShell(isShell), voltageDelay(voltageDelay), pressureDelay(pressureDelay), tempHumidityDelay(tempHumidityDelay), servoDelay(servoDelay) {
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
                
                for (Voltage v: voltageList) {
                    v.callVoltage();
                }
            }

            if (currentTime - prevTimePressure >= pressureDelay) {
                prevTimePressure = currentTime;

                for (Pressure p: pressureList) {
                    p.callPressure();
                }
            }

            if (currentTime - prevTimeTempHumidity >= tempHumidityDelay) {
                prevTimeTempHumidity = currentTime;

                for (TempHumidity th: tempHumidityList) {
                    th.callTempHumidity();
                }
            }

            if (currentTime - prevTimeServo >= servoDelay) {
                prevTimeServo = currentTime;

                if (Serial.available() > 0) {
                    string input = Serial.readString();
                    string id = input.substr(0,1);
                    string data = input.substr(2);

                    if (servoMap.count(id)) {
                        Servo s = ServoMap.at(id);
                        if ((s.getMinPWM < data) && (data < s.getMaxPWM))
                            s.callServo(data);
                    }
                }
            }
        }
}