#pragma once

#include "main.h"

#include <vector>

struct motorConfiguration {
    int port = 1;
    bool reversed = false;
};

class MotorGroup
{
public:
    MotorGroup(std::vector<motorConfiguration> motors);
    ~MotorGroup();

    void move(int32_t voltage);
    void move_velocity(int32_t velocity);
    void set_gearing(pros::motor_gearset_e_t gearset);
    void set_brake_mode(pros::motor_brake_mode_e_t mode);
    void brake();
    
private:
    std::vector<pros::Motor *> motors;
};