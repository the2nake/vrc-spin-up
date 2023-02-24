/**
 * Filename: HolonomicXDrive.cpp
 * Author: home.vn2007@gmail.com
 * Copyright (c) 2023 by home.vn2007@gmail.com
 * All rights reserved
*/

#include "HolonomicXDrive.hpp"

#include "utility-functions.hpp"

#include "main.h"

HolonomicXDrive::HolonomicXDrive(int portFrontLeft, int portFrontRight, int portBackRight, int portBackLeft, int portImu)
{
    this->mFL = new pros::Motor(portFrontLeft);
    this->mFR = new pros::Motor(portFrontRight, 1);
    this->mBR = new pros::Motor(portBackRight, 1);
    this->mBL = new pros::Motor(portBackLeft);

    this->imu = new pros::Imu(portImu);

    this->setBrakeMode(MOTOR_BRAKE_BRAKE);

    this->mFL->set_gearing(MOTOR_GEAR_200);
    this->mFR->set_gearing(MOTOR_GEAR_200);
    this->mBR->set_gearing(MOTOR_GEAR_200);
    this->mBL->set_gearing(MOTOR_GEAR_200);
}

HolonomicXDrive::~HolonomicXDrive()
{
    pros::Motor *motors[] = {mFL, mFR, mBR, mBL};
    for (auto motor : motors)
    {
        delete motor;
    }

    delete imu;
}

void HolonomicXDrive::setBrakeMode(pros::motor_brake_mode_e_t mode)
{
    for (auto i : {mFL, mFR, mBR, mBL})
    {
        i->set_brake_mode(mode);
    }
}

void HolonomicXDrive::drive(double vTrans, double hTrans)
{
    driveAndTurn(vTrans, hTrans, 0);
}

void HolonomicXDrive::driveAndTurn(double vTrans, double hTrans, double vRot)
{
    if (vTrans == 0 && vRot == 0)
    {
        return;
    }

    double mTrans = std::abs(vTrans);

    double dRot = (vRot < 0 ? -1 : 1);
    double mRot = std::abs(vRot);

    double theta;
    if (vTrans < 0)
    {
        theta = findMod(180 + hTrans, 360);
    }
    else
    {
        theta = findMod(hTrans, 360);
    }
    theta = findMod(theta - imu->get_heading(), 360);

    double vFL = cosDeg(405 - theta) * mTrans * mTrans / (mTrans + mRot) + dRot * mRot * mRot / (mTrans + mRot);
    double vBR = cosDeg(405 - theta) * mTrans * mTrans / (mTrans + mRot) - dRot * mRot * mRot / (mTrans + mRot);
    double vFR = cosDeg(675 - theta) * mTrans * mTrans / (mTrans + mRot) - dRot * mRot * mRot / (mTrans + mRot);
    double vBL = cosDeg(675 - theta) * mTrans * mTrans / (mTrans + mRot) + dRot * mRot * mRot / (mTrans + mRot);

    // if max(vFL, vBR, vFR, vBL) < 1, multiply each of them by 1/max(vFL, vBR, vFR, vBL

    // double velocities[] = {vFL, vFR, vBR, vBL};

    // double maxSpeed = std::abs(*std::max_element(std::begin(velocities), std::end(velocities)));
    // double targetSpeed = std::min(mTrans + mRot, 1.0);
    //
    // vFL *= targetSpeed / maxSpeed;
    // vFR *= targetSpeed / maxSpeed;
    // vBR *= targetSpeed / maxSpeed;
    // vBL *= targetSpeed / maxSpeed;

    this->mFL->move_velocity(200.0 * vFL);
    this->mFR->move_velocity(200.0 * vFR);
    this->mBR->move_velocity(200.0 * vBR);
    this->mBL->move_velocity(200.0 * vBL);
}

void HolonomicXDrive::brake()
{
    for (auto i : {mFL, mFR, mBR, mBL})
    {
        i->brake();
    }
}