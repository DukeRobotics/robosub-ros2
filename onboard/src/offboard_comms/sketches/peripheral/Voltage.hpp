#include <Arduino.h>

class Voltage {
    private:
        int pinNum;
        float voltage;
        float onboardVoltage;
        String voltageTag;

    public:
        Voltage(int pinNum, float onboardVoltage, String tagSuffix) : pinNum(pinNum), onboardVoltage(onboardVoltage) {
            voltageTag = "V" + tagSuffix + ":";
        }

        void callVoltage() {
            voltage = analogRead(pinNum);
            voltage = voltage*onboardVoltage/1023*5; // From datasheet, for analog to digital conversion
            String printVoltage = this->voltageTag + String(voltage);
            Serial.println(printVoltage);
        }
};