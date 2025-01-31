#include <Arduino.h>
class Voltage {
    private:
        int pinNum;
        float voltage;
        float onboardVoltage;
        String voltageTag;

    public:
        Voltage(int pinNum, float onboardVoltage) : pinNum(pinNum), onboardVoltage(onboardVoltage) {
            voltageTag = "V: ";
        }

        void callVoltage() {
            voltage = analogRead(pinNum);
            voltage = voltage*onboardVoltage/1023*5; // from datasheet, for analog to digital conversion
            String printVoltage = this->voltageTag + String(voltage);
            Serial.println(printVoltage);
        }
};