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

void StarDrive::driveAndMaintainHeading(double translationVelocity, double translationHeading, double rotationHeading, double threshold)
{
    double currentHeading = this->odometry->getAbsolutePosition().heading;
    // sanitse data
    currentHeading = findMod(currentHeading, 360.0);
    rotationHeading = findMod(rotationHeading, 360.0);
    // calculate angle to turn right and left
    double deltaRight = findMod(rotationHeading - currentHeading, 360.0);
    double delta = 0.0;
    if (deltaRight > 180.0)
    {
        delta = deltaRight;
    }
    else
    {
        delta = -deltaRight - 360.0;
    }
    double output = 0.0;
    if (delta > std::abs(threshold))
    {
        output = std::min(sinDeg(std::min(90.0, delta)), 0.1); // sine function, max velocity if delta >= 90
    }
    else if (delta < -std::abs(threshold))
    {
        output = std::max(sinDeg(std::max(delta, -90.0)), -0.1); // sine is an odd function, so will result in a negative turn
    }
    this->driveAndTurn(translationVelocity, translationHeading, output);
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

    double velocities[] = {vFL, vFR, vMR, vBR, vBL, vML};
    double min_mult = 600.0;

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
            else if (cast_ptr->get_gearing() == MOTOR_GEAR_600 || cast_ptr->get_pto_mode())
            {
                mult = 600.0;
            }
            if (mult < min_mult)
            {
                min_mult = mult;
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
            if (mult < min_mult)
            {
                min_mult = mult;
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
            cast_ptr->move_velocity(std::min(min_mult, this->maxRPM * velocities[i]));
        }
        else
        {
            m->move_velocity(std::min(min_mult, this->maxRPM * velocities[i]));
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
    if (rpm > 0 && rpm < 600)
    {
        this->maxRPM = rpm;
    }
}

double StarDrive::getMaxRPM()
{
    return this->maxRPM;
}
