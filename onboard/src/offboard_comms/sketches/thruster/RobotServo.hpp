#include <Arduino.h>
#include <Servo.h>

struct ServoConfig {
    uint8_t pinNum;
    uint16_t minPWM;
    uint16_t stopPWM;
    uint16_t maxPWM;
    uint16_t tag;
    int delay; // Delay
};

class RobotServo {
    private:
        uint8_t pinNum;
        uint16_t minPWM;
        uint16_t stopPWM;
        uint16_t maxPWM;
        uint16_t tag;
        int delay; // Delay in milliseconds for servo to return to stop position
        Servo myServo;
        bool servoActive = false;
        unsigned long servoTime;

    public:
        RobotServo(){}

        RobotServo(uint8_t pinNum, uint16_t minPWM, uint16_t stopPWM, uint16_t maxPWM, uint16_t tag, int delay=1000) :
        pinNum(pinNum), minPWM(minPWM), stopPWM(stopPWM), maxPWM(maxPWM), tag(tag), delay(delay) {
            
        }

        void begin() {
            myServo.attach(pinNum);
            myServo.writeMicroseconds(stopPWM);
        }

        uint16_t getTag() {
            return tag;
        }

        void stopServo() {
            myServo.writeMicroseconds(stopPWM);
            servoActive = false;
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
            // If the servo is active and the delay has passed, return the servo to the stop position
            if (servoActive && (millis() - servoTime >= delay)) {
                stopServo();
            }
        }
};
