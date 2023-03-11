#pragma once

#include "main.h"

class PTOMotor : public pros::Motor
{
public:
    PTOMotor(const std::int8_t port, const bool reverse = false, const pros::motor_gearset_e_t gearset = MOTOR_GEAR_GREEN,
             const pros::motor_encoder_units_e_t encoder_units = MOTOR_ENCODER_DEGREES);
    ~PTOMotor();

    void set_pto_mode(bool mode);
    bool get_pto_mode();

    std::int32_t move_velocity(const std::int32_t velocity) const override;
    std::int32_t move_velocity_if_pto(const std::int32_t velocity) const;
    std::int32_t move_velocity_override(const std::int32_t velocity) const;

    std::int32_t brake(void) const override;
    std::int32_t brake_if_pto(void) const;
    std::int32_t brake_override(void) const;

    std::int32_t set_brake_mode(const pros::motor_brake_mode_e_t mode) const override;

private:
    pros::Motor *motor = nullptr;
    bool pto_mode = false; // false = drive, true = non-drive
};