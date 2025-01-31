#include "Robot.hpp"

#define VOLTAGE_PIN 3
#define TH_PIN 4
#define SERVO_MARKER 8
#define SERVO_TORPEDO 9

class Oogway : public Robot
{
    private:
        bool isShell;
        int voltageDelay;
        int pressureDelay;
        int tempHumidityDelay;
        int servoDelay;

        String tempHumidityTagPrefix;
        String servo_marker_tag;
        String servo_torpedo_tag;

        Voltage voltage_sensor;
        Pressure pressure_sensor;
        TempHumidity temp_humidity_sensor;
        RobotServo servo_marker;
        RobotServo servo_torpedo;
    public:
        Oogway(int voltageDelay, int pressureDelay, int tempHumidityDelay, int servoDelay, bool isShell = false) :
        voltageDelay(voltageDelay), pressureDelay(pressureDelay), tempHumidityDelay(tempHumidityDelay), servoDelay(servoDelay), Robot(isShell) {
            tempHumidityTagPrefix = "oogway";

            voltage_sensor = new Voltage(VOLTAGE_PIN, 4.655);
            pressure_sensor = new Pressure();
            temp_humidity_sensor = new TempHumidity(TH_PIN), ;
            servo_marker = new RobotServo(SERVO_MARKER, 1100, 1500, 1900);
            servo_torpedo = new RobotServo(SERVO_TORPEDO, 1100, 1500, 1900);

            servo_marker_tag = "M";
            servo_torpedo_tag = "T";

            servoMap.insert({servo_marker_tag, servo_marker});
            servoMap.insert({servo_torpedo_tag, servo_torpedo});

            voltageList.insert(voltage_sensor);

            pressureList.insert(pressure_sensor);

            tempHumidityList.insert(temp_humidity_sensor);
        }
};