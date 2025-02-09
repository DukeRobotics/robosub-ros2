#include <Arduino.h>
#include "DHT11.h"

class TempHumidity {
    private:
        int pinNum;
        String humidityTag;
        String tempTag;
        DHT11* dht11;

    public:
        TempHumidity(int pinNum, String tagSuffix) : pinNum(pinNum) {
            humidityTag = "H" + tagSuffix + ":";
            tempTag = "T" + tagSuffix + ":";

            dht11 = new DHT11(pinNum);
            dht11->setDelay(0);
        }

        void callTempHumidity() {
            int temperature = 0;
            int humidity = 0;
            int result = dht11->readTemperatureHumidity(temperature, humidity);

            // If result is 0, then the read was successful
            // If result is not 0, then the read was unsuccessful; do not print any data and try reading again next time this function is called
            if (result == 0) {
                String printHumidity = this->humidityTag + String((float)humidity);
                Serial.println(printHumidity);

                String printTemp = this->tempTag + String((float)temperature * 1.8 + 32); // Convert Celsius to Fahrenheit
                Serial.println(printTemp);
            }
        }
};