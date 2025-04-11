#include <Arduino.h>

class Voltage {
    private:
        int pinNum;
        float onboardVoltage;
        float slope;
        float intercept;
        String voltageTag;
        float voltage;

    public:
        Voltage(int pinNum, float onboardVoltage, float slope, float intercept, String tagSuffix) : pinNum(pinNum), onboardVoltage(onboardVoltage), slope(slope), intercept(intercept) {
            voltageTag = "V" + tagSuffix + ":";
        }

        void callVoltage() {
            voltage = analogRead(pinNum);
            voltage = voltage * onboardVoltage / 1023 * 5; // From datasheet, for analog to digital conversion
            voltage = voltage * slope + intercept; // Apply linear regression to correct for sensor error
            String printVoltage = this->voltageTag + String(voltage);
            Serial.println(printVoltage);
        }
};