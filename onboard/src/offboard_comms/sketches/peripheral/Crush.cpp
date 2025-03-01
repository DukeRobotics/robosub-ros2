#include "Robot.hpp"

#define VOLTAGE_PIN 3
#define BTH_PIN 2
#define CTH_PIN 4
//#define SERVO_A 8

class Crush : public Robot {
private:
    // Flag to determine if this is Crush's shell (true) or the actual robot (false)
    bool isShell;

    Voltage* voltage_sensor;
    Pressure* pressure_sensor;
    TempHumidity* battery_temp_humidity_sensor;
    TempHumidity* comp_temp_humidity_sensor;
    //RobotServo* servo_a;

public:
    Crush(int voltageDelay, int pressureDelay, int tempHumidityDelay, int servoDelay, bool isShell = false)
        : Robot(voltageDelay, pressureDelay, tempHumidityDelay, servoDelay), isShell(isShell) {

        voltage_sensor = new Voltage(VOLTAGE_PIN, 4.655, "");
        pressure_sensor = new Pressure("");
        battery_temp_humidity_sensor = new TempHumidity(BTH_PIN, "B");
        comp_temp_humidity_sensor = new TempHumidity(CTH_PIN, "C");
        //servo_a = new RobotServo(SERVO_A, 500, 1500, 2500, "M");




        addVoltageSensor(voltage_sensor);
        addPressureSensor(pressure_sensor);
        addTempHumiditySensor(battery_temp_humidity_sensor);
        addTempHumiditySensor(computer_temp_humidity_sensor);
        addServo(servo_a);
    }
};
