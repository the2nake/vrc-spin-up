#pragma once

#include "main.h"


/**
 * A field-based control framework for a holonomic X drive
*/
class HolonomicXDrive
{

public:
    HolonomicXDrive() {}
    HolonomicXDrive(int portFrontLeft, int portFrontRight, int portBackRight, int portBackLeft, int portImu);

    void setBrakeMode(pros::motor_brake_mode_e_t mode);

    void drive(double translateVelocity, double translateHeading);
    void driveAndTurn(double translateVelocity, double translateHeading, double rotationVelocity);
    
    void brake();

private:
    pros::Motor *mFL;
    pros::Motor *mFR;
    pros::Motor *mBR;
    pros::Motor *mBL;

    pros::Imu *imu;
};