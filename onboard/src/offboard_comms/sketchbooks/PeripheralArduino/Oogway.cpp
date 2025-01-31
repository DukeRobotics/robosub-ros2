#include "Robot.hpp"
#include <string>

#define VOLTAGE_PIN 3
#define TH_PIN 4
#define SERVO_MARKER 8
#define SERVO_TORPEDO 9

class Oogway : public Robot
{
    public:
        Oogway(bool isShell) : Robot(isShell) {
            Voltage voltage_sensor = new Voltage(VOLTAGE_PIN);
            Pressure pressure_sensor = new Pressure();
            TempHumidity temp_humidity = new TempHumidity(TH_PIN);
            Servo servo_marker = new Servo(SERVO_MARKER, 1100, 1500, 1900);
            Servo servo_torpedo = new Servo(SERVO_TORPEDO, 1100, 1500, 1900);

            std::string servo_marker_tag = "M";
            std::string servo_torpedo_tag = "T";

            servoMap.insert({servo_marker_tag, servo_marker});
            servoMap.insert({servo_torpedo_tag, servo_torpedo});

            voltageList.insert(voltage_sensor);

            pressureList.insert(pressure_sensor);

            tempHumidityList.insert(temp_humdity);
        }
}