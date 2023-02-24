/**
 * Filename: MotorGroup.hpp
 * Author: home.vn2007@gmail.com
 * Copyright (c) 2023 by home.vn2007@gmail.com
 * All rights reserved
*/

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
    void set_encoder_units(pros::motor_encoder_units_e_t units);
    void set_brake_mode(pros::motor_brake_mode_e_t mode);
    void brake();

    double get_actual_velocity();
    double get_temperature();
    
private:
    std::vector<pros::Motor *> motors;
};