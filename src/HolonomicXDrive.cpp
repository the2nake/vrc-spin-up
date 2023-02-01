#include "HolonomicXDrive.hpp"

#include "utility-functions.hpp"

#include "main.h"

HolonomicXDrive::HolonomicXDrive(int portFrontLeft, int portFrontRight, int portBackRight, int portBackLeft, int portImu)
{
    this->mFL = new pros::Motor(portFrontLeft);
    this->mFR = new pros::Motor(portFrontRight);
    this->mBR = new pros::Motor(portBackRight);
    this->mBL = new pros::Motor(portBackLeft);

    this->imu = new pros::Imu(portImu);

    this->setBrakeMode(MOTOR_BRAKE_BRAKE);
}

void HolonomicXDrive::setBrakeMode(pros::motor_brake_mode_e_t mode)
{
    for (auto i : {mFL, mFR, mBR, mBL})
    {
        i->set_brake_mode(mode);
    }
}

void HolonomicXDrive::drive(double vTrans, double aTrans)
{
    driveAndTurn(vTrans, aTrans, 0);
}

void HolonomicXDrive::driveAndTurn(double vTrans, double aTrans, double vRot)
{
    double mTrans = vTrans;
    if (vTrans < 0)
    {
        mTrans = 0 - mTrans;
        aTrans += 180;
    }

    double dRot = !(vRot < 0);
    double mRot = std::abs(vRot / 2.0); // scale rotation down

    double theta = findMod(360 + 90 - aTrans, 360);
    theta = findMod(theta - imu->get_heading() + 360, 360);

    double vTPosSlope = cosDeg(findMod(405 - theta, 360)) * mTrans * mTrans;
    double vTNegSlope = cosDeg(findMod(675 - theta, 360)) * mTrans * mTrans;
    double scaledVRot = dRot * mRot * mRot;

    double vFL = vTPosSlope / (mTrans + mRot) + scaledVRot / (mTrans + mRot);
    double vFR = vTNegSlope / (mTrans + mRot) - scaledVRot / (mTrans + mRot);
    double vBR = vTPosSlope / (mTrans + mRot) - scaledVRot / (mTrans + mRot);
    double vBL = vTNegSlope / (mTrans + mRot) + scaledVRot / (mTrans + mRot);

    this->mFL->move(127.0 * vFL);
    this->mFR->move(127.0 * vFR);
    this->mBR->move(127.0 * vBR);
    this->mBL->move(127.0 * vBL);
}

void HolonomicXDrive::driveAndTurnToHeading(double vTrans, double aTrans, double heading)
{
    heading = findMod(360 + heading, 360);
    double deltaThetaLeft = findMod(360 + this->imu->get_heading() - heading, 360);
    double deltaThetaRight = findMod(360 - deltaThetaLeft, 360);

    if (deltaThetaLeft < deltaThetaRight)
    {
        driveAndTurn(vTrans, aTrans, -0.75*deltaThetaLeft/180.0);
    } else {
        driveAndTurn(vTrans, aTrans, 0.75*deltaThetaRight/180.0);
    }
}

void HolonomicXDrive::brake()
{
    for (auto i : {mFL, mFR, mBR, mBL})
    {
        i->brake();
    }
}