/**
 * Filename: XDrive.hpp
 * Author: home.vn2007@gmail.com
 * Copyright (c) 2023 by home.vn2007@gmail.com
 * All rights reserved
 */

#pragma once

#include "main.h"

/**
 * A field-based control framework for an X drive
 */
class XDrive
{

public:
    XDrive() {}

    /**
     * Creates a XDrive object that controls an X Drive
     *
     * @param portFrontLeft: the port on the V5 Brain that the front-left motor is connected to
     * @param portFrontRight: the port on the V5 Brain that the front-right motor is connected to
     * @param portBackRight: the port on the V5 Brain that the back-right motor is connected to
     * @param portBackLeft: the port on the V5 Brain that the back-left motor is connected to
     *
     * @param portImu: the port on the V5 Brain that the Inertial Sensor is connected to
     */
    XDrive(int portFrontLeft, int portFrontRight, int portBackRight, int portBackLeft, int portImu);

    ~XDrive();

    /**
     * Sets the braking mode for each motor in the drivetrain
     *
     * @param mode: one of MOTOR_BRAKE_{COAST, BRAKE, HOLD}
     */
    virtual void setBrakeMode(pros::motor_brake_mode_e_t mode);

    /**
     * Translates the X Drive following the specified heading and relative velocity
     *
     * @param translateVelocity 0 <= translateVelocity <= 1
     * @param translateHeading -360 <= translateHeading <= 360
     */
    virtual void drive(double translateVelocity, double translateHeading);

    /**
     * Translates the X Drive following the specified heading and relative velocity,
     *   and turn clockwise at the relative velocity provided at the same time
     *
     * @param translateVelocity 0 <= translateVelocity <= 1
     * @param translateHeading -360 <= translateHeading <= 360
     * @param rotationalVelocity -1 <= rotationalVelocity <= 1
     */
    virtual void driveAndTurn(double translateVelocity, double translateHeading, double rotationVelocity);

    /**
     * Brakes all of the motors in the drivetrain according to the current braking mode
     */
    virtual void brake();

private:
    pros::Motor *mFL;
    pros::Motor *mFR;
    pros::Motor *mBR;
    pros::Motor *mBL;

    pros::Imu *imu;
};