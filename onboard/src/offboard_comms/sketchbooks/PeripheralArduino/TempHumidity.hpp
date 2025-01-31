#include <Arduino.h>
#include "DHT11.h"

class TempHumidity {
    private:
        int pinNum;
        String humidityTag;
        String tempTag;
        DHT11* dht11; // Use a pointer instead of a direct instance

    public:
        TempHumidity(int pinNum, String tagSuffix) {
            this->pinNum = pinNum;
            humidityTag = "H" + tagSuffix + ":";
            tempTag = "T" + tagSuffix + ":";

            dht11 = new DHT11(pinNum); // Dynamically allocate the DHT11 object
            dht11->setDelay(0);
        }

        ~TempHumidity() {
            delete dht11; // Free allocated memory
        }

        void callTempHumidity() {
            int temperature = 0;
            int humidity = 0;
            int result = dht11->readTemperatureHumidity(temperature, humidity);

            if (result == 0) {
                String printHumidity = this->humidityTag + String((float)humidity);
                Serial.println(printHumidity);

                String printTemp = this->tempTag + String((float)temperature * 1.8 + 32); // convert Celsius to Fahrenheit
                Serial.println(printTemp);
            }
        }
};