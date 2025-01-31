#include <string>
#include <Arduino.h>
#include <Wire.h>
#include "MS5837.h"

class Pressure {
    private:
        int pinNum;
        bool pressureConnected;
        string MS5837 sensor;
    public:
        Pressure(int pinNum) : pinNum(pinNum) {
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
            sensor.setModel(MS5837::MS5837_02BA);
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
                String printPressure = PRESSURETAG + String(sensor.depth());
                Serial.println(printPressure);
            }

            // If sensor.read had an error but did not time out, try reading again in next loop
        }

        // If pressure sensor is disconnected, try to reinitalize it
        else {
            initPressure();
        }
    }
}