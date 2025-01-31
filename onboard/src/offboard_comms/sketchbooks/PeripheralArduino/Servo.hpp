#include <Arduino.h>
#include <Servo.h>

class Servo {
    private:
        int pinNum;
        int minPWM;
        int stopPWM;
        int maxPWM;
        int delay; // delay in miliseconds for servo to return to stop position
        Servo myServo;
        bool servoMoved = false;
        unsigned long servoTime;

    public:
        Servo(int pinNum, int minPWM, int stopPWM, int maxPWM, int delay=1000) : pinNum(pinNum), minPWM(minPWM), stopPWM(stopPWM), maxPWM(maxPWM), delay(delay) {
            myServo.attach(pinNum);
            myServo.writeMicroseconds(1500);
        }

        void callServo(int direction) {
            unsigned long currentTime = millis();
            myservo.writeMicroseconds(direction);  // 1200 = 90 degrees left  // 1250 micros =  20 write
            servoMoved = true;
            servoTime = currentTime;

            // return to default position after delay milisecond
            if(servoMoved && ((myTime - servoTime) > delay)) {
                myservo.writeMicroseconds(1500);  // 1500 micros = 90 write
                servoMoved = false;
            }
        }

        int getMinPWM() {
            return minPWM;
        }

        int getMaxPWM() {
            return maxPWM;
        }
}