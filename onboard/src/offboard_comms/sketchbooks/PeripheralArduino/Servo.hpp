#include <Arduino.h>
#include <Servo.h>

#define LEFT 1250
#define RIGHT 1750

class Servo {
    private:
        int pinNum;
        Servo myServo;
        bool servoMoved = false;
        unsigned long servoTime;
        float ONBOARD_VOLTAGE = 4.655;

    public:
        Servo(int pinNum) : pinNum(pinNum) {
            myServo.attach(pinNum);
            myServo.writeMicroseconds(1500);

        }

        void callServo(int direction) {
            unsigned long currentTime = millis();
            myservo.writeMicroseconds(direction);  // 1200 = 90 degrees left  // 1250 micros =  20 write
            servoMoved = true;
            servoTime = currentTime;
        }
}