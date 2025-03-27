#include "Robot.hpp"

#define VOLTAGE_PIN 3
#define THB_PIN 2
#define THS_PIN 4

class Crush : public Robot {
private:
    // Flag to determine if this is Crush's shell (true) or the actual robot (false)
    bool isShell;

    Voltage* voltage_sensor;
    Pressure* pressure_sensor;
    TempHumidity* battery_temp_humidity_sensor;
    TempHumidity* signal_temp_humidity_sensor;

public:
    Crush(int voltageDelay, int pressureDelay, int tempHumidityDelay, int servoDelay, bool isShell = false)
        : Robot(voltageDelay, pressureDelay, tempHumidityDelay, servoDelay), isShell(isShell) {

        voltage_sensor = new Voltage(VOLTAGE_PIN, 4.655, "");
        pressure_sensor = new Pressure(MS5837::MS5837_30BA, "");
        battery_temp_humidity_sensor = new TempHumidity(THB_PIN, "B");
        signal_temp_humidity_sensor = new TempHumidity(THS_PIN, "S");

        addVoltageSensor(voltage_sensor);
        addPressureSensor(pressure_sensor);
        addTempHumiditySensor(battery_temp_humidity_sensor);
        addTempHumiditySensor(signal_temp_humidity_sensor);
    }
};
