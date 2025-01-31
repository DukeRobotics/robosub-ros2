#include <Arduino.h>
class Voltage {
    private:
        int pinNum;
        const string voltageTag = "V: ";
        float voltage;


    public:
        Voltage(int pinNum) : pinNum(pinNum) {}

        void callVoltage() {
            voltage = analogRead(pinNum);
            voltage = voltage*ONBOARD_VOLTAGE/1023*5; // from datasheet, for analog to digital conversion
            String printVoltage = voltageTag + String(voltage);
            Serial.println(printVoltage);
        }
}