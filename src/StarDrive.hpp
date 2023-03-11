#pragma once

#include "XDrive.hpp"
#include "APS.hpp"

#include "main.h"

class StarDrive : public XDrive {
public:
    StarDrive(pros::Motor *frontLeft, pros::Motor *frontRight, pros::Motor *midRight, pros::Motor *backRight, pros::Motor*backLeft, pros::Motor *midLeft, APS* odometry);

    void drive(double translationVelocity, double translationHeading) override;
    void driveAndMaintainHeading(double translationVelocity, double translationHeading, double rotationHeading);

    void driveAndTurn(double translationVelocity, double translationHeading, double rotationVelocity) override;

    void setBrakeMode(pros::motor_brake_mode_e_t mode) override;
    void brake() override;

private:
    pros::Motor *frontLeft;
    pros::Motor *frontRight;
    pros::Motor *midRight;
    pros::Motor *backRight;
    pros::Motor *backLeft;
    pros::Motor *midLeft;

    bool isPTO[6] = {false, false, false, false, false, false};

    APS* odometry;
};