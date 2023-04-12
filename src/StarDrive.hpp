#pragma once

#include "XDrive.hpp"
#include "APS.hpp"
#include "PIDController.hpp"

#include "main.h"

#include <chrono>

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
    void driveAndMaintainHeading(double translationVelocity, double translationHeading, double rotationHeading, double threshold = 2.0);

    void driveAndTurn(double translationVelocity, double translationHeading, double rotationVelocity) override;

    void setBrakeMode(pros::motor_brake_mode_e_t mode) override;
    void brake() override;

    void setMaxRPM(double rpm);
    double getMaxRPM();
    void setOutputRPMs(std::vector<double> rpms);
    std::vector<double> getOutputRPMs();

    void configTranslationalPID(PIDConfig config);
    void configAngularPID(PIDConfig config);

    void initiateMotionTo(absolutePosition targetPose);
    void setMotionTarget(absolutePosition targetPose);
    void moveFollowingMotionProfile();
    void haltPIDMotion();

    bool isPIDActive() {
        return this->PIDActive;
    }

private:
    pros::Motor *frontLeft = nullptr;
    pros::Motor *frontRight = nullptr;
    pros::Motor *midRight = nullptr;
    pros::Motor *backRight = nullptr;
    pros::Motor *backLeft = nullptr;
    pros::Motor *midLeft = nullptr;

    APS* odometry = nullptr;
    
    double maxRPM = 600.0;
    std::vector<double> outputRPMs = {200.0, 200.0, 200.0, 200.0, 200.0, 200.0}; // modifiers representing external gearing

    /* Profiled motion */
    bool PIDActive = false;
    absolutePosition targetPose;

    PIDController *angularVelocityController = nullptr;
    PIDController *translationalVelocityController = nullptr;

    PIDConfig angularConfig;
    PIDConfig translationalConfig;
};