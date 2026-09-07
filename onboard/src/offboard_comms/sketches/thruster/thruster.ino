#include <Arduino.h>
#include "FrameParser.hpp"
#include "Actuators.hpp"


constexpr uint32_t BAUD_RATE = 57600;
Actuators actuators = Actuators();
FrameParser parser = FrameParser(NUM_THRUSTERS);


void setup(){
    Serial.begin(BAUD_RATE);

    actuators.setup();

}

void loop(){
    actuators.updateAll();
    while (Serial.available()) {
        uint8_t data = Serial.read();
        if (parser.processByte(data)) {
            DeviceType type;
            uint16_t buffer[32];
            if (parser.getFrame(type, buffer)) {
                if (type == DeviceType::Thruster) {
                    // Process thruster data
                    actuators.applyThrusterPwms(buffer);

                } else if (type == DeviceType::Servo) {
                    // Process servo data
                    actuators.applyServoPwms(buffer[0], buffer[1]);
                }
            }
        }
    }
}

