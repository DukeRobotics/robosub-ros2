#include "Robot.hpp"

#define VOLTAGE_PIN 3
#define TH_PIN 4
#define SERVO_MARKER 8
#define SERVO_TORPEDO 9

class Oogway : public Robot {
private:
    // Flag to determine if this is Oogway's shell (true) or the actual robot (false)
    bool isShell;

    Voltage* voltage_sensor;
    Pressure* pressure_sensor;
    TempHumidity* temp_humidity_sensor;
    RobotServo* servo_marker;
    RobotServo* servo_torpedo;

public:
    Oogway(int voltageDelay, int pressureDelay, int tempHumidityDelay, int servoDelay, bool isShell = false)
        : Robot(voltageDelay, pressureDelay, tempHumidityDelay, servoDelay), isShell(isShell) {

        voltage_sensor = new Voltage(VOLTAGE_PIN, 4.655, 0.987, -0.00524, "");
        pressure_sensor = new Pressure(MS5837::MS5837_02BA, "");
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
