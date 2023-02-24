/**
 * Filename: MotorGroup.cpp
 * Author: home.vn2007@gmail.com
 * Copyright (c) 2023 by home.vn2007@gmail.com
 * All rights reserved
*/

#include "MotorGroup.hpp"

#include "main.h"

#include <vector>

MotorGroup::MotorGroup(std::vector<motorConfiguration> motorConfigs)
{
    for (auto config : motorConfigs)
    {
        this->motors.push_back(new pros::Motor(config.port, config.reversed));
    }
}

MotorGroup::~MotorGroup()
{
    for (auto motor : this->motors)
    {
        delete motor;
    }
}

void MotorGroup::move(int32_t voltage)
{
    for (pros::Motor *motor : this->motors)
    {
        motor->move(voltage);
    }
}

void MotorGroup::move_velocity(int32_t velocity)
{
    for (pros::Motor *motor : this->motors)
    {
        motor->move_velocity(velocity);
    }
}

void MotorGroup::set_gearing(pros::motor_gearset_e_t gearset)
{

    for (pros::Motor *motor : this->motors)
    {
        motor->set_gearing(gearset);
    }
}

void MotorGroup::set_encoder_units(pros::motor_encoder_units_e_t units)
{

    for (pros::Motor *motor : this->motors)
    {
        motor->set_encoder_units(units);
    }
}

void MotorGroup::set_brake_mode(pros::motor_brake_mode_e_t mode)
{

    for (pros::Motor *motor : this->motors)
    {
        motor->set_brake_mode(mode);
    }
}

void MotorGroup::brake()
{
    for (pros::Motor *motor : this->motors)
    {
        motor->brake();
    }
}

double MotorGroup::get_actual_velocity() {
    double sum = 0;
    for (pros::Motor *motor : this->motors)
    {
        sum += motor->get_actual_velocity();
    }

    return sum / (double)(this->motors.size());
}

double MotorGroup::get_temperature() {
    double sum = 0;
    for (pros::Motor *motor : this->motors)
    {
        sum += motor->get_temperature();
    }

    return sum / (double)(this->motors.size());
}
