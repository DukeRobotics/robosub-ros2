#define VPIN 3
#define VOLTAGE_PERIOD 100
#define VOLTAGETAG "V:"

class Voltage {
    private:
        int pinNum;
        string voltagetag;


    public:
    Voltage(int pinNum, string voltagetag) {
        voltagetag = "V:"
    }

    void callVoltage() {
        currentTime = millis();
        if(currentTime - prevTimeVoltage > VOLTAGE_PERIOD) {
            prevTimeVoltage = currentTime;

            voltage = analogRead(VPIN);
            voltage = voltage*ONBOARD_VOLTAGE/1023*5; // from datasheet, for analog to digital conversion
            String printVoltage = VOLTAGETAG + String(voltage);
            Serial.println(printVoltage);
        }
    }
}