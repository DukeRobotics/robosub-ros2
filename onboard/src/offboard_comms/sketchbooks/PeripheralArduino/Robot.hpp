#include "Servo.hpp"
#include "TempHumidity.hpp"
#include <map>
#include <string>
#include <vector>

#define extractOpCode(x) ((x & 0b11100000) >> 5)
#define extractServoPin(x) ((x & 0b00011110) >> 1)
#define extractDirection(x) (x & 0b00000001)


class Robot {
    private:
        bool isShell;
        std::map<int, Servo> servoMap;
        std::map<int, TempHumidity> thMap;


    public:
        Robot(bool isShell = false) : isShell(isShell) {}

        void init();
        void process() {
            if (Serial.available() > 0) {
                string input = Serial.readString();
                string id = input.substr(0,1);
                string data = input.substr(2);

                if(servoMap.count(id)){
                    Servo s = ServoMap.at(id);
                    s.callServo(data);
                }


                // // command 111 = Servo
                // byte command = extractOpCode(input);
                // byte pin = extractServoPin(input);
                // byte direction = extractDirection(input); // 0 = left, 1 = right

                // switch (command)
                // {
                // case 111:
                //     Servo s = servoMap.at(pin);
                //     s.callServo(direction);
                //     break;

                // default:
                //     break;
                // }
            }
        }
}