#include "StarDrive.hpp"

#include "PTOMotor.hpp"
#include "utility-functions.hpp"

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

void StarDrive::driveAndMaintainHeading(double translationVelocity, double translationHeading, double rotationHeading)
{
}

void StarDrive::driveAndTurn(double translationVelocity, double translationHeading, double rotationVelocity)
{
    if (translationVelocity == 0 && rotationVelocity == 0)
    {
        return;
    }

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

    double vFL = cosDeg(405 - theta) * translationSpeed * translationSpeed / (translationSpeed + rotationSpeed) + rotationDirection * rotationSpeed * rotationSpeed / (translationSpeed + rotationSpeed);
    double vBR = cosDeg(405 - theta) * translationSpeed * translationSpeed / (translationSpeed + rotationSpeed) - rotationDirection * rotationSpeed * rotationSpeed / (translationSpeed + rotationSpeed);
    double vFR = cosDeg(675 - theta) * translationSpeed * translationSpeed / (translationSpeed + rotationSpeed) - rotationDirection * rotationSpeed * rotationSpeed / (translationSpeed + rotationSpeed);
    double vBL = cosDeg(675 - theta) * translationSpeed * translationSpeed / (translationSpeed + rotationSpeed) + rotationDirection * rotationSpeed * rotationSpeed / (translationSpeed + rotationSpeed);

    double vML = cosDeg(theta) * translationSpeed * translationSpeed / (translationSpeed + rotationSpeed) + 0.70710678118 * rotationDirection * rotationSpeed * rotationSpeed / (translationSpeed + rotationSpeed);
    double vMR = cosDeg(theta) * translationSpeed * translationSpeed / (translationSpeed + rotationSpeed) - 0.70710678118 * rotationDirection * rotationSpeed * rotationSpeed / (translationSpeed + rotationSpeed);

    double velocities[] = {200.0 * vFL, 200.0 * vFR, 200.0 * vMR, 200.0 * vBR, 200.0 * vBL, 200.0 * vML};

    int i = 0;
    for (auto m : {frontLeft, frontRight, midRight, backRight, backLeft, midLeft})
    {
        if (dynamic_cast<PTOMotor *>(m) != nullptr)
        {
            dynamic_cast<PTOMotor *>(m)->move_velocity(velocities[i]);
        }
        else
        {
            m->move_velocity(velocities[i]);
        }
        i++;
    }
}

void StarDrive::setBrakeMode(pros::motor_brake_mode_e_t mode)
{
    int i = 0;
    for (auto m : {frontLeft, frontRight, midRight, backRight, backLeft, midLeft})
    {
        if (dynamic_cast<PTOMotor *>(m) != nullptr)
        {
            dynamic_cast<PTOMotor *>(m)->set_brake_mode(mode);
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
        if (dynamic_cast<PTOMotor *>(m) != nullptr)
        {
            dynamic_cast<PTOMotor *>(m)->brake();
        }
        else
        {
            m->brake();
        }
        i++;
    }
}