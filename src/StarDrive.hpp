#pragma once

#include "XDrive.hpp"
#include "APS.hpp"

#include "main.h"

class StarDrive : public XDrive {
public:
    StarDrive(pros::Motor *frontLeft, pros::Motor *frontRight, pros::Motor *midRight, pros::Motor *backRight, pros::Motor*backLeft, pros::Motor *midLeft, APS* odometry);

    void drive(double translationVelocity, double translationHeading) override;

    /**
     * Drives along the translation vector, attempting to maintain a heading of rotationHeading
     * 
     * Will only devote, at maximum, 9% of the drivetrain's power to maintaining the angle
     * Optionally has a threshold argument to avoid overcompensation
    */
    void driveAndMaintainHeading(double translationVelocity, double translationHeading, double rotationHeading, double threshold = 0.5);

    void driveAndTurn(double translationVelocity, double translationHeading, double rotationVelocity) override;

    void setBrakeMode(pros::motor_brake_mode_e_t mode) override;
    void brake() override;

    void setMaxRPM(double rpm);
    double getMaxRPM();
    
    void setOutputRPMs(std::vector<double> rpms);
    std::vector<double> getOutputRPMs();

private:
    pros::Motor *frontLeft;
    pros::Motor *frontRight;
    pros::Motor *midRight;
    pros::Motor *backRight;
    pros::Motor *backLeft;
    pros::Motor *midLeft;

    APS* odometry;
    
    double maxRPM = 600.0;

    std::vector<double> outputRPMs = {200.0, 200.0, 200.0, 200.0, 200.0, 200.0}; // modifiers representing external gearing
};