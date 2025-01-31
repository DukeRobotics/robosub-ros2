#include <Arduino.h>
#include <Servo.h>

class RobotServo {
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
        RobotServo(int pinNum, int minPWM, int stopPWM, int maxPWM, int delay=1000) :
        pinNum(pinNum), minPWM(minPWM), stopPWM(stopPWM), maxPWM(maxPWM), delay(delay) {
            myServo.attach(pinNum);
            myServo.writeMicroseconds(stopPWM);
        }

        void callServo(int pwm) {
            // Make sure pwm is within the min and max PWM values
            if (pwm < minPWM || pwm > maxPWM) {
                return;
            }

            unsigned long currentTime = millis();
            myServo.writeMicroseconds(pwm);  // 1200 = 90 degrees left  // 1250 micros =  20 write
            servoMoved = true;
            servoTime = currentTime;

            // Return to default position after delay
            if(servoMoved && ((currentTime - servoTime) > delay)) {
                myServo.writeMicroseconds(stopPWM);  // 1500 micros = 90 write
                servoMoved = false;
            }
        }

        int getMinPWM() {
            return minPWM;
        }

        int getMaxPWM() {
            return maxPWM;
        }
};