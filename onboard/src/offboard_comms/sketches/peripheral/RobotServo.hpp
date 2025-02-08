#include <Arduino.h>
#include <Servo.h>

class RobotServo {
    private:
        int pinNum;
        int minPWM;
        int stopPWM;
        int maxPWM;
        String tag;
        int delay; // Delay in milliseconds for servo to return to stop position
        Servo myServo;
        bool servoActive = false;
        unsigned long servoTime;

    public:
        RobotServo(int pinNum, int minPWM, int stopPWM, int maxPWM, String tag, int delay=1000) :
        pinNum(pinNum), minPWM(minPWM), stopPWM(stopPWM), maxPWM(maxPWM), tag(tag), delay(delay) {
            myServo.attach(pinNum);
            myServo.writeMicroseconds(stopPWM);
        }

        String getTag() {
            return tag;
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