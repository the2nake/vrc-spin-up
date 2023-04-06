/**
 * Filename: APS.cpp
 * Author: home.vn2007@gmail.com
 * Copyright (c) 2023 by home.vn2007@gmail.com
 * All rights reserved
 */

#include "APS.hpp"

#include "main.h"
#include "utility-functions.hpp"

APS::APS(encoderConfig leftEncoderConfig, encoderConfig rightEncoderConfig, encoderConfig strafeEncoderConfig,
         double sLO, double sOR, double sOS, odomConfig wheelODs, pros::Imu *imu, double imuWeight)
{
    this->leftEnc = new pros::ADIEncoder(leftEncoderConfig.topPort, leftEncoderConfig.bottomPort, leftEncoderConfig.reversed);
    this->rightEnc = new pros::ADIEncoder(rightEncoderConfig.topPort, rightEncoderConfig.bottomPort, rightEncoderConfig.reversed);
    this->strafeEnc = new pros::ADIEncoder(strafeEncoderConfig.topPort, strafeEncoderConfig.bottomPort, strafeEncoderConfig.reversed);

    if (errno == ENXIO || errno == ENODEV)
    {
        this->imuWeight = 1.0;
        this->encodersDisabled = true;
    }

    this->leftWheelOD = wheelODs.leftWheelOD;
    this->rightWheelOD = wheelODs.rightWheelOD;
    this->strafeWheelOD = wheelODs.strafeWheelOD;
    this->sLO = sLO;
    this->sOR = sOR;
    this->sOS = sOS;

    if (imu != nullptr)
    {
        this->imu = imu;
        if (!this->encodersDisabled)
        {
            this->imuWeight = imuWeight;
        }
    }
    else
    {
        if (this->encodersDisabled)
        {
            pros::screen::print(TEXT_LARGE_CENTER, 3, "APS Module Malfunction"); // TODO: file bug in pros kernel to fix TEXT_LARGE_CENTER macro
        }
        this->imu = nullptr;
        this->imuWeight = 0.0;
    }
}

APS::~APS()
{
    delete this->leftEnc;
    delete this->rightEnc;
    delete this->strafeEnc;
}

void APS::setAbsolutePosition(double x, double y, double heading)
{
    while (!this->positionDataMutex.take(5))
    {
    }

    if (!this->encodersDisabled)
    {
        this->leftEnc->reset();
        this->rightEnc->reset();
        this->strafeEnc->reset();
    }

    if (x != APS_NO_CHANGE)
    {
        this->absX = x;
    }
    if (y != APS_NO_CHANGE)
    {
        this->absY = y;
    }

    if (heading != APS_NO_CHANGE)
    {
        this->absHeading = heading;
        if (this->imu != nullptr)
        {
            this->imu->set_heading(heading);
        }
    }

    positionDataMutex.give();
}

void APS::updateAbsolutePosition()
{
    /**
     * This function should be called at least once every 10 milliseconds for sufficient accuracy
     */

    if (this->encodersDisabled)
    {
        if (imuWeight == 0.0)
        {
            return;
        }
        else
        {
            while (!this->positionDataMutex.take(5))
            {
            }
            this->absHeading = imu->get_heading();
            this->positionDataMutex.give();
        }
        return;
    }

    int currLeftEncVal = this->leftEnc->get_value();
    int currRightEncVal = this->rightEnc->get_value();
    int currStrafeEncVal = this->strafeEnc->get_value();

    double dL = (currLeftEncVal - this->prevLeftEncVal) * this->leftWheelOD / 360.0;
    double dR = (currRightEncVal - this->prevRightEncVal) * this->rightWheelOD / 360.0;
    double dS = (currStrafeEncVal - this->prevStrafeEncVal) * this->strafeWheelOD / 360.0;

    double prevHeading = this->absHeading;

    // calculated as an absolute quantity
    double newHeading = (180 / 3.141592) * ((currLeftEncVal / 360.0) * this->leftWheelOD - (currRightEncVal / 360.0) * this->rightWheelOD) / (this->sLO + this->sOR);
    if (0 < this->imuWeight && this->imuWeight <= 1)
    {
        newHeading = (newHeading * (1 - this->imuWeight) + this->imuWeight * this->imu->get_heading());
    }
    // find dX, dY, dH from dL, dR, and dS
    // dY is the along the axis from the previous position to this position
    // dX is the along the axis perpendicular to that

    double dH = newHeading - prevHeading;
    double dX = 180.0 * dS / (dH * 3.141592) - sOS;
    double dY = (180.0 * dR / (dH * 3.141592) + sOR + 180.0 * dL / (dH * 3.141592) + sLO) / 2.0;

    dX *= 2 * sinDeg(dH / 2.0);
    dY *= 2 * sinDeg(dH / 2.0);

    // use a rotation matrix on dX and dY, anticlockwise dH degrees

    dX = dX * cosDeg(newHeading) - dY * sinDeg(newHeading);
    dY = dX * sinDeg(newHeading) + dY * cosDeg(newHeading);

    // make sure nothing writes to these variables
    // setting a few variables should never take more than one update cycle
    while (!this->positionDataMutex.take(5))
    {
    }
    this->absX = this->absX + dX;
    this->absY = this->absY + dY;
    this->absHeading = newHeading;
    this->positionDataMutex.give();

    this->prevLeftEncVal = currLeftEncVal;
    this->prevRightEncVal = currRightEncVal;
    this->prevStrafeEncVal = currStrafeEncVal;
}

absolutePosition APS::getAbsolutePosition()
{
    double x = this->absX.load();
    double y = this->absY.load();
    double h = this->absHeading.load();

    return {x, y, h};
}