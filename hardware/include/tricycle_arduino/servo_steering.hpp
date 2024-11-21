#ifndef TRICYCLE_ARDUINO_STEERING_HPP
#define TRICYCLE_ARDUINO_STEERING_HPP

#include <string>
#include <math.h>

class ServoSteering
{
private:
    double minControlValue = -0.5;
    double maxControlValue = 0.5;
    double minArduinoValue = 45;
    double maxArduinoValue = 135;
    double controlToArduinoSlope = (maxArduinoValue - minArduinoValue) / (maxControlValue - minControlValue);
    double arduinoToControlSlope = (maxControlValue - minControlValue) / (maxArduinoValue - minArduinoValue);

    double clamp(double value, double min, double max) const
    {
        return std::max(min, std::min(value, max));
    }

public:
    std::string name = "";
    double cmd = 0;
    double cmd_raw = 0;
    double pos = 0;
    double pos_raw = 0;


    ServoSteering() = default;

    ServoSteering(const std::string &joint_name)
    {
        name = joint_name;
    }

    void map_cmd_to_arduino()
    {
        cmd_raw = cmd * 1000;
    }

    void map_pos_to_control()
    {
        pos = pos_raw / 1000;
    }
};

#endif // TRICYCLE_ARDUINO_STEERING_HPP