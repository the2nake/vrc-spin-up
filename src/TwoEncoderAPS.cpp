/**
 * Filename: TwoEncoderAPS.cpp
 * Author: home.vn2007@gmail.com
 * Copyright (c) 2023 by home.vn2007@gmail.com
 * All rights reserved
 */

#include "TwoEncoderAPS.hpp"

#include "APS.hpp"

#include "main.h"
#include "utility-functions.hpp"

TwoEncoderAPS::TwoEncoderAPS(encoderConfig yc, encoderConfig xc, double syo, double sox, odomConfig wheelODs, pros::Imu *imu, double imuMultiplier)
{
    this->yEnc = new pros::ADIEncoder(yc.topPort, yc.bottomPort, yc.reversed);
    this->xEnc = new pros::ADIEncoder(xc.topPort, xc.bottomPort, xc.reversed);

    if (errno == ENXIO || errno == ENODEV)
    {
        this->encodersDisabled = true;
    }

    this->yWheelOD = wheelODs.leftWheelOD;
    this->xWheelOD = wheelODs.strafeWheelOD;
    this->sYO = syo;
    this->sOX = sox;

    this->imuMultiplier = imuMultiplier;

    if (imu != nullptr)
    {
        this->imu = imu;
    }
    else
    {
        if (this->encodersDisabled)
        {
            pros::screen::print(TEXT_LARGE_CENTER, 3, "APS Module Malfunction"); // TODO: file bug in pros kernel to fix TEXT_LARGE_CENTER macro
        }
        this->imu = nullptr;
    }
}

TwoEncoderAPS::~TwoEncoderAPS()
{
    delete this->yEnc;
    delete this->xEnc;
}

void TwoEncoderAPS::setAbsolutePosition(double x, double y, double heading)
{
    while (!this->positionDataMutex.take(5))
    {
    }

    if (!this->encodersDisabled)
    {
        this->yEnc->reset();
        this->xEnc->reset();
        this->prevXEncVal = this->xEnc->get_value();
        this->prevYEncVal = this->yEnc->get_value();
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
        this->prevHeading = heading;
        if (this->imu != nullptr)
        {
            this->imu->set_heading(heading);
            previousImuHeading = heading;
        }
    }

    positionDataMutex.give();
}

void TwoEncoderAPS::updateAbsolutePosition()
{
    /**
     * This function should be called at least once every 10 milliseconds for sufficient accuracy
     */

    if (this->encodersDisabled)
    {
        if (imu == nullptr)
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

    int currYEncVal = this->yEnc->get_value();
    int currXEncVal = this->xEnc->get_value();
    double currentImuHeading = imu->get_heading();

    double dYEnc = (currYEncVal - this->prevYEncVal) * this->yWheelOD / 360.0;
    double dXEnc = (currXEncVal - this->prevXEncVal) * this->xWheelOD / 360.0;

    // if (dYEnc == 0 && dXEnc == 0) {
    //     return;
    // }

    // find dX, dY, dH from dL, dR, and dS
    // dY is the along the axis from the previous position to this position
    // dX is the along the axis perpendicular to that

    double dH = findShorterTurn(previousImuHeading, currentImuHeading, 360.0) * imuMultiplier;
    double dY, dX;
    if (dH == 0)
    {
        dY = dYEnc;
        dX = dXEnc;
    }
    else
    {
        dY = 180.0 * dYEnc / (dH * 3.141592) + sYO;
        dX = 180.0 * dXEnc / (dH * 3.141592) + sOX;

        dX *= 2 * sinDeg(dH / 2.0);
        dY *= 2 * sinDeg(dH / 2.0);
    }

    // calculated as an absolute quantity
    double newHeading = prevHeading + dH;

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
    this->absHeading = findMod(newHeading, 360.0);
    this->positionDataMutex.give();

    this->prevYEncVal = currYEncVal;
    this->prevXEncVal = currXEncVal;
    this->prevHeading = this->absHeading;
    this->previousImuHeading = currentImuHeading;
}

absolutePosition TwoEncoderAPS::getAbsolutePosition()
{
    double x = this->absX.load();
    double y = this->absY.load();
    double h = this->absHeading.load();

    return {x, y, h};
}