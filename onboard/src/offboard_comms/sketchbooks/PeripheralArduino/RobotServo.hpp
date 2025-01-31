#include <Arduino.h>
#include <Servo.h>

class RobotServo {
    private:
        int pinNum;
        int minPWM;
        int stopPWM;
        int maxPWM;
        int delay; // Delay in milliseconds for servo to return to stop position
        Servo myServo;
        bool servoActive = false;
        unsigned long servoTime;

    public:
        RobotServo(int pinNum, int minPWM, int stopPWM, int maxPWM, int delay=1000) :
        pinNum(pinNum), minPWM(minPWM), stopPWM(stopPWM), maxPWM(maxPWM), delay(delay) {
            myServo.attach(pinNum);
            myServo.writeMicroseconds(stopPWM);
        }

        void callServo(int pwm) {
            // Reject new commands while the servo is in motion
            if (servoActive) {
                return;
            }

            // Make sure pwm is within the min and max PWM values
            if (pwm < minPWM || pwm > maxPWM) {
                return;
            }

            myServo.writeMicroseconds(pwm);
            servoActive = true;
            servoTime = millis();
        }

        void updateServo() {
            if (servoActive && (millis() - servoTime >= delay)) {
                myServo.writeMicroseconds(stopPWM);
                servoActive = false;
            }
        }
};