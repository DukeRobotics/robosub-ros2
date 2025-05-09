#include <Arduino.h>
#include <Wire.h>
#include "MS5837.h"

class Pressure {
    private:
        uint8_t sensorModel;
        bool pressureConnected;
        String pressureTag;
        MS5837 sensor;
    public:
        Pressure(uint8_t model, String tagSuffix) {
            sensorModel = model;
            pressureTag = "P" + tagSuffix + ":";
            pressureConnected = false;
            initPressure();
        }

        void initPressure() {
            Wire.end();
            Wire.begin();
            Wire.setWireTimeout(500, true);

            sensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)

            pressureConnected = sensor.init();
            if (pressureConnected) {
                sensor.setModel(sensorModel);
            }
        }

        void callPressure() {
            // If pressure sensor is connected, read the pressure
            if (pressureConnected) {
                byte error = sensor.read();

                // If sensor.read timed out, mark pressure sensor as disconnected and clear the timeout flag
                if (error == 5) {
                    pressureConnected = false;
                    Wire.clearWireTimeoutFlag();
                }

                // If sensor.read was successful, print the pressure
                if (!error) {
                    Serial.flush();

                    String printPressure = this->pressureTag + String(sensor.depth());
                    Serial.println(printPressure);
                }

                // If sensor.read had an error but did not time out, try reading again in next call
            }

            // If pressure sensor is disconnected, try to reinitalize it
            else {
                initPressure();
            }
        }
};