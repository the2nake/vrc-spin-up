#include "PTOMotor.hpp"

PTOMotor::PTOMotor(const std::int8_t port, const pros::motor_gearset_e_t gearset, const bool reverse,
                   const pros::motor_encoder_units_e_t encoder_units) : pros::Motor(port, gearset, reverse, encoder_units)
{
    this->motor = new pros::Motor(port, gearset, reverse, encoder_units);
    this->pto_mode = false;
}

PTOMotor::~PTOMotor()
{
    delete this->motor;
}

void PTOMotor::set_pto_mode(bool mode)
{
    this->pto_mode = mode;
}

bool PTOMotor::get_pto_mode()
{
    return this->pto_mode;
}

std::int32_t PTOMotor::move_velocity(const std::int32_t velocity) const
{
    if (!pto_mode)
    {
        return this->motor->move_velocity(velocity);
    }
    return 0;
}

std::int32_t PTOMotor::move_velocity_if_pto(const std::int32_t velocity) const
{
    if (pto_mode)
    {
        return this->motor->move_velocity(velocity);
    }
    return 0;
}

std::int32_t PTOMotor::move_velocity_override(const std::int32_t velocity) const
{
    return this->motor->move_velocity(velocity);
}

std::int32_t PTOMotor::brake(void) const
{
    if (!pto_mode)
    {
        return this->motor->brake();
    }
    return 0;
}

std::int32_t PTOMotor::brake_if_pto(void) const
{
    if (pto_mode)
    {
        return this->motor->brake();
    }
    return 0;
}

std::int32_t PTOMotor::brake_override(void) const
{
    return this->motor->brake();
}

std::int32_t PTOMotor::set_brake_mode(const pros::motor_brake_mode_e_t mode) const {
    return this->motor->set_brake_mode(mode);
}