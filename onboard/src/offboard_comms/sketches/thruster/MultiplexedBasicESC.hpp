// This library was developed by the Duke Robotics Club.
// It is designed to control a Blue Robotics Basic ESC using the Adafruit PWM Servo Driver.

#include <Arduino.h>
#include "Adafruit_PWMServoDriver.h"

class MultiplexedBasicESC{
    private:
        uint8_t pin;
        bool is_attached;
        Adafruit_PWMServoDriver *multiplexer;
    public:
        MultiplexedBasicESC() {
            is_attached = false;
        }

        ~MultiplexedBasicESC() {
            detach();
        }

        /*
         * Initialize the ESC with the PWM multiplexer
         * @param _multiplexer: The PWM multiplexer to use
        */
        void initialize(Adafruit_PWMServoDriver *_multiplexer) {
            multiplexer = _multiplexer;
        }

        /*
         * Attach the ESC to the PWM multiplexer
         * @param _pin: The pin on the PWM multiplexer to attach the ESC to
        */
        void attach(uint8_t _pin) {
            // Detach the ESC if it is already attached
            if(is_attached) {
                detach();
            }
            is_attached = true;
            pin = _pin;
            multiplexer->setPWMFreq(250);
        }

        /*
         * Check if the ESC is attached to the PWM multiplexer
         * @return: True if the ESC is attached, false otherwise
        */
        bool attached() {
            return is_attached;
        }

        /*
         * Detach the ESC from the PWM multiplexer
        */
        void detach() {
            is_attached = false;
            multiplexer->setPin(pin, 0, false);
        }

        /*
         * Write a PWM value to the ESC
         * @param uS: The PWM value in microseconds
        */
        void write(uint16_t uS) {
            if(is_attached) {
                multiplexer->setPin(pin, map(uS, 1100, 1900, 1000, 1720), false);
            }
        }
};
