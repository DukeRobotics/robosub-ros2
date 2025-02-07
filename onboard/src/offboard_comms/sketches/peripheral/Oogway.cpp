#include "Robot.hpp"

#define VOLTAGE_PIN 3
#define TH_PIN 4
#define SERVO_MARKER 8
#define SERVO_TORPEDO 9

class Oogway : public Robot {
private:
    bool isShell;
    int voltageDelay;
    int pressureDelay;
    int tempHumidityDelay;
    int servoDelay;

    Voltage* voltage_sensor;
    Pressure* pressure_sensor;
    TempHumidity* temp_humidity_sensor;
    RobotServo* servo_marker;
    RobotServo* servo_torpedo;

public:
    Oogway(int voltageDelay, int pressureDelay, int tempHumidityDelay, int servoDelay, bool isShell = false)
        : Robot(voltageDelay, pressureDelay, tempHumidityDelay, servoDelay), isShell(isShell) {

        voltage_sensor = new Voltage(VOLTAGE_PIN, 4.655, "");
        pressure_sensor = new Pressure("");
        temp_humidity_sensor = new TempHumidity(TH_PIN, "");
        servo_marker = new RobotServo(SERVO_MARKER, 500, 1500, 2500, "M");
        servo_torpedo = new RobotServo(SERVO_TORPEDO, 700, 1500, 2300, "T");

        addVoltageSensor(voltage_sensor);
        addPressureSensor(pressure_sensor);
        addTempHumiditySensor(temp_humidity_sensor);
        addServo(servo_marker);
        addServo(servo_torpedo);
    }
};
