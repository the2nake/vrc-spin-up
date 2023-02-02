#pragma once

#include "main.h"


/**
 * A field-based control framework for a holonomic X drive
*/
class HolonomicXDrive
{

public:
    HolonomicXDrive() {}

    /**
     * Creates a HolonomicXDrive object that controls an X Drive
     * 
     * portFrontLeft: the port on the V5 Brain that the front-left motor is connected to
     * portFrontRight: the port on the V5 Brain that the front-right motor is connected to
     * portBackRight: the port on the V5 Brain that the back-right motor is connected to
     * portBackLeft: the port on the V5 Brain that the back-left motor is connected to
     * 
     * portImu: the port on the V5 Brain that the Inertial Sensor is connected to
    */
    HolonomicXDrive(int portFrontLeft, int portFrontRight, int portBackRight, int portBackLeft, int portImu);

    /**
     * Sets the braking mode for each motor in the drivetrain
     * 
     * mode: one of MOTOR_BRAKE_COAST, MOTOR_BRAKE_BRAKE, or MOTOR_BRAKE_HOLD
    */
    void setBrakeMode(pros::motor_brake_mode_e_t mode);

    /**
     * Translates the X Drive following the specified heading and relative velocity
     * 
     * 0 <= translateVelocity <= 1
     * -360 <= translateHeading <= 360
    */
    void drive(double translateVelocity, double translateHeading);

    /**
     * Translates the X Drive following the specified heading and relative velocity,
     *   and turn clockwise at the relative velocity provided at the same time
     * 
     * 0 <= translateVelocity <= 1
     * -360 <= translateHeading <= 360
     * -1 <= rotationalVelocity <= 1
    */
    void driveAndTurn(double translateVelocity, double translateHeading, double rotationVelocity);
    
    /**
     * Brakes all of the motors in the drivetrain according to the current braking mode
    */
    void brake();

private:
    pros::Motor *mFL;
    pros::Motor *mFR;
    pros::Motor *mBR;
    pros::Motor *mBL;

    pros::Imu *imu;
};