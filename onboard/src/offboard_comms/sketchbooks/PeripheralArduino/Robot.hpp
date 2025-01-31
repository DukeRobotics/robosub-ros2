#include "Servo.hpp"
#include "TempHumidity.hpp"
#include <map>
#include <string>
#include <vector>

class Robot {
    protected:
        bool isShell;
        std::vector<Voltage> voltageList;
        std::vector<Pressure> pressureList;
        std::vector<TempHumidity> tempHumidityList;
        std::map<std::string, Servo> servoMap;

    public:
        Robot(bool isShell = false) : isShell(isShell) {}

        void init();

        void process() {
            for (Voltage v: voltageList) {
                v.callVoltage();
            }
            for (Pressure p: pressureList) {
                p.callPressure();
            }
            for (TempHumidity th: tempHumidityList) {
                th.callTempHumidity();
            }

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