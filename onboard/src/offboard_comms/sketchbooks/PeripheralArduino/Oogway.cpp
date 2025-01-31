#include "Robot.hpp"

#define VOLTAGE_PIN 3
#define TH_PIN 4
#define SERVO_MARKER 8
#define SERVO_TORPEDO 9

class Oogway : public Robot
{
    private:
        std::string tempHumidityTagPrefix;
        std::string servo_marker_tag;
        std::string servo_torpedo_tag;

        Voltage voltage_sensor;
        Pressure pressure_sensor;
        TempHumidity temp_humidity_sensor;
        Servo servo_marker;
        Servo servo_torpedo;
    public:
        Oogway(bool isShell = false, int voltageDelay, int pressureDelay, int tempHumidityDelay, int servoDelay) :
        Robot(isShell), voltageDelay(voltageDelay), pressureDelay(pressureDelay), tempHumidityDelay(tempHumidityDelay), servoDelay(servoDelay) {
            tempHumidityTagPrefix = "oogway";

            voltage_sensor = new Voltage(VOLTAGE_PIN);
            pressure_sensor = new Pressure();
            temp_humidity_sensor = new TempHumidity(TH_PIN), ;
            servo_marker = new Servo(SERVO_MARKER, 1100, 1500, 1900);
            servo_torpedo = new Servo(SERVO_TORPEDO, 1100, 1500, 1900);

            servo_marker_tag = "M";
            servo_torpedo_tag = "T";

            servoMap.insert({servo_marker_tag, servo_marker});
            servoMap.insert({servo_torpedo_tag, servo_torpedo});

            voltageList.insert(voltage_sensor);

            pressureList.insert(pressure_sensor);

            tempHumidityList.insert(temp_humdity_sensor);
        }
}