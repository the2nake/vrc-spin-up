#include "StarDrive.hpp"

#include "PTOMotor.hpp"
#include "utility-functions.hpp"

#include <cmath>

StarDrive::StarDrive(pros::Motor *frontLeft, pros::Motor *frontRight, pros::Motor *midRight, pros::Motor *backRight, pros::Motor *backLeft, pros::Motor *midLeft, APS *odometry)
{
    // motors should be set up so that fwd moves the robot in the fwd direction

    this->frontLeft = frontLeft;
    this->frontRight = frontRight;
    this->midRight = midRight;
    this->backRight = backRight;
    this->backLeft = backLeft;
    this->midLeft = midLeft;

    this->odometry = odometry;
}

void StarDrive::drive(double translationVelocity, double translationHeading)
{
    this->driveAndTurn(translationVelocity, translationHeading, 0);
}

void StarDrive::driveAndMaintainHeading(double translationVelocity, double translationHeading, double rotationHeading, double threshold)
{
    double currentHeading = this->odometry->getAbsolutePosition().heading;
    // sanitse data
    currentHeading = findMod(currentHeading, 360.0);
    rotationHeading = findMod(rotationHeading, 360.0);
    // calculate angle to turn right and left
    double deltaRight = findMod(rotationHeading - currentHeading, 360.0);
    double delta = findShorterTurn(currentHeading, rotationHeading, 360.0);
    double output = 0.0;
    if (delta > std::abs(threshold))
    {
        output = std::max(sinDeg(std::min(90.0, delta)) / 8.0, 0.05); // sine function, max velocity if delta >= 90
    }
    else if (delta < -std::abs(threshold))
    {
        output = std::min(sinDeg(std::max(delta, -90.0)) / 8.0, -0.05); // sine is an odd function, so will result in a negative turn
    }

    if (translationVelocity == 0.0 && output == 0.0)
    {
        this->brake();
    }
    else
    {
        this->driveAndTurn(translationVelocity, translationHeading, output);
    }
}

void StarDrive::driveAndTurn(double translationVelocity, double translationHeading, double rotationVelocity)
{
    if (translationVelocity == 0 && rotationVelocity == 0)
    {
        this->brake();
        return;
    }

    /* Actual turning portion */

    double translationSpeed = std::abs(translationVelocity);

    double rotationDirection = (rotationVelocity < 0 ? -1 : 1);
    double rotationSpeed = std::abs(rotationVelocity);

    double theta;
    if (translationVelocity < 0)
    {
        theta = findMod(180 + translationHeading, 360);
    }
    else
    {
        theta = findMod(translationHeading, 360);
    }
    theta = findMod(theta - this->odometry->getAbsolutePosition().heading, 360);

    auto translateScale = translationSpeed * translationSpeed / (translationSpeed + rotationSpeed);
    auto rotationScale = rotationSpeed * rotationSpeed / (translationSpeed + rotationSpeed);

    double vFL = cosDeg(405 - theta) * translateScale + rotationDirection * rotationScale;
    double vBR = cosDeg(405 - theta) * translateScale - rotationDirection * rotationScale;
    double vFR = cosDeg(675 - theta) * translateScale - rotationDirection * rotationScale;
    double vBL = cosDeg(675 - theta) * translateScale + rotationDirection * rotationScale;

    double vML = cosDeg(theta) * translationSpeed * translationSpeed / (translationSpeed + rotationSpeed) + 0.70710678118 * rotationDirection * rotationSpeed * rotationSpeed / (translationSpeed + rotationSpeed);
    double vMR = cosDeg(theta) * translationSpeed * translationSpeed / (translationSpeed + rotationSpeed) - 0.70710678118 * rotationDirection * rotationSpeed * rotationSpeed / (translationSpeed + rotationSpeed);

    double velocities[] = {vFL, vFR, vMR, vBR, vBL, vML};
    double min_max_rpm = 600.0;

    int i = 0;
    for (auto m : {frontLeft, frontRight, midRight, backRight, backLeft, midLeft})
    {
        PTOMotor *cast_ptr = dynamic_cast<PTOMotor *>(m);
        if (cast_ptr != nullptr)
        {
            double mult = 100.0;
            if (cast_ptr->get_gearing() == MOTOR_GEAR_200)
            {
                mult = 200.0;
            }
            else if (cast_ptr->get_gearing() == MOTOR_GEAR_600)
            {
                mult = 600.0;
            }
            else if (cast_ptr->get_pto_mode())
            {
                continue;
            }
            if (mult < min_max_rpm)
            {
                min_max_rpm = mult;
            }
        }
        else
        {
            double mult = 100.0;
            if (m->get_gearing() == MOTOR_GEAR_200)
            {
                mult = 200.0;
            }
            else if (m->get_gearing() == MOTOR_GEAR_600)
            {
                mult = 600.0;
            }
            if (mult < min_max_rpm)
            {
                min_max_rpm = mult;
            }
        }
        i++;
    }

    i = 0;
    for (auto m : {frontLeft, frontRight, midRight, backRight, backLeft, midLeft})
    {
        PTOMotor *cast_ptr = dynamic_cast<PTOMotor *>(m);
        if (cast_ptr != nullptr)
        {
            cast_ptr->move_velocity(std::min(min_max_rpm, (this->maxRPM * velocities[i]) * rpmFromGearset(cast_ptr->get_gearing()) / outputRPMs[i]));
        }
        else
        {
            m->move_velocity(std::min(min_max_rpm, (this->maxRPM * velocities[i]) * rpmFromGearset(m->get_gearing()) / outputRPMs[i]));
        }
        i++;
    }
}

void StarDrive::setBrakeMode(pros::motor_brake_mode_e_t mode)
{
    int i = 0;
    for (auto m : {frontLeft, frontRight, midRight, backRight, backLeft, midLeft})
    {
        PTOMotor *cast_ptr = dynamic_cast<PTOMotor *>(m);
        if (cast_ptr != nullptr)
        {
            cast_ptr->set_brake_mode(mode);
        }
        else
        {
            m->set_brake_mode(mode);
        }
        i++;
    }
}

void StarDrive::brake()
{
    int i = 0;
    for (auto m : {frontLeft, frontRight, midRight, backRight, backLeft, midLeft})
    {
        PTOMotor *cast_ptr = dynamic_cast<PTOMotor *>(m);
        if (cast_ptr != nullptr)
        {
            cast_ptr->brake();
        }
        else
        {
            m->brake();
        }
        i++;
    }
}

void StarDrive::setMaxRPM(double rpm)
{
    if (rpm > 0 && rpm < vectorMax<double>(outputRPMs))
    {
        this->maxRPM = rpm;
    }
}

double StarDrive::getMaxRPM()
{
    return this->maxRPM;
}

void StarDrive::setOutputRPMs(std::vector<double> rpms)
{
    if (rpms.size() == 6)
    {
        this->outputRPMs = rpms;
    }
}

std::vector<double> StarDrive::getOutputRPMs()
{
    return this->outputRPMs;
}

void StarDrive::configTranslationalPID(PIDConfig config)
{
    this->translationalConfig = config;
    if (this->translationalVelocityController == nullptr)
    {
        this->translationalVelocityController = new PIDController(config);
    }
    else
    {
        this->translationalVelocityController->setPIDConstants(config);
    }
}

void StarDrive::configAngularPID(PIDConfig config)
{
    this->angularConfig = config;
    if (this->angularVelocityController == nullptr)
    {
        this->angularVelocityController = new PIDController(config);
    }
    else
    {
        this->angularVelocityController->setPIDConstants(config);
    }
}

void StarDrive::setMotionTarget(absolutePosition targetPose)
{
    this->targetPose = targetPose;
    if (translationalVelocityController != nullptr)
    {
        this->translationalVelocityController->startPID(0.0); // use PID for distance target 0.0
    }
    if (angularVelocityController != nullptr)
    {
        this->angularVelocityController->startPID(0.0); // use PID for delta theta target 0.0
    }
    this->PIDActive = true;
}

void StarDrive::moveFollowingMotionProfile()
{
    if (!PIDActive)
    {
        this->brake();
        return;
    }

    // get the current position
    auto currentPose = this->odometry->getAbsolutePosition();
    auto dX = this->targetPose.x - currentPose.x;
    auto dY = this->targetPose.y - currentPose.y;
    auto translationVector = polarFromCartesian(dX, dY);

    double translationalOutput = 0.0, angularOutput = 0.0;

    // update the PID outputs
    if (translationalVelocityController != nullptr)
    {
        translationalOutput = this->translationalVelocityController->updatePID(-translationVector.rho); // negative so PID output is positive
    }
    if (angularVelocityController != nullptr)
    {
        angularOutput = this->angularVelocityController->updatePID(-findShorterTurn(currentPose.heading, targetPose.heading, 360.0)); // negative so PID output is positive
    }

    // BUG: integral will not be cut when robot crosses the target point, so integral will increase indefinitely
    // NOTE: you cannot simply set the angle from the beginning because the drivetrain will drift over time
    // TODO: choose between disabling the integral, or cutting the angles of approach in half so that the angles facing the original point will have positive distance values and
    //       angles facing away will have negative distance values
    // NOTE: for drivetrains, integral may not be necessary if the inertia of the robot is sufficient

    // if settled
    if (std::abs(translationalOutput) < 0.005 && std::abs(angularOutput) < 0.005)
    {
        PIDActive = false;
        if (translationalVelocityController != nullptr)
        {
            this->translationalVelocityController->resetPIDSystem();
        }

        if (angularVelocityController != nullptr)
        {
            this->angularVelocityController->resetPIDSystem();
        }
        this->brake();
        return;
    }

    // move if not there yet
    double translationVelocity = std::min(std::max(translationalOutput, 0.0), 1.0);
    double translationHeading = findMod(90 - translationVector.theta, 360.0);
    double angularVelocity = std::min(std::max(angularOutput, -1.0), 1.0);
    this->driveAndTurn(translationVelocity, translationHeading, angularVelocity);
}

void StarDrive::haltPIDMotion()
{
    this->PIDActive = false;
    if (translationalVelocityController != nullptr)
    {
        this->translationalVelocityController->resetPIDSystem();
    }
    if (angularVelocityController != nullptr)
    {
        this->angularVelocityController->resetPIDSystem();
    }
}
