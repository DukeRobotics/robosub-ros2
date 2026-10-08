#include <Arduino.h>
#include "DHT.h"

class TempHumidity {
    private:
        int pinNum;
        String humidityTag;
        String tempTag;
        DHT* dht22;

    public:
        TempHumidity(int pinNum, String tagSuffix) : pinNum(pinNum) {
            humidityTag = "H" + tagSuffix + ":";
            tempTag = "T" + tagSuffix + ":";

            dht22 = new DHT(pinNum, DHT22);
            dht22->begin();
        }

        void callTempHumidity() {
            float temperature = dht22->readTemperature(true, true); // Fahrenheit and Force (Can Collect Within 2 Seconds)
            float humidity = dht22->readHumidity(true); // Force (Can Collect Within 2 Seconds)

            // If result is 0, then the read was successful
            // If result is not 0, then the read was unsuccessful; do not print any data and try reading again next time this function is called
            if (!(isnan(temperature) || isnan(humidity))) {
                String printHumidity = this->humidityTag + String((float)humidity);
                Serial.println(printHumidity);

                String printTemp = this->tempTag + String((float)temperature); // Fahrenheit
                Serial.println(printTemp);
            }
        }
};