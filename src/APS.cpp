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
         double sLO, double sOR, double sOS, double wheelSize)
{
    this->leftEnc = new pros::ADIEncoder(leftEncoderConfig.topPort, leftEncoderConfig.bottomPort, leftEncoderConfig.reversed);
    this->rightEnc = new pros::ADIEncoder(rightEncoderConfig.topPort, rightEncoderConfig.bottomPort, rightEncoderConfig.reversed);
    this->strafeEnc = new pros::ADIEncoder(strafeEncoderConfig.topPort, strafeEncoderConfig.bottomPort, strafeEncoderConfig.reversed);

    this->wheelSize = wheelSize;
    this->sLO = sLO;
    this->sOR = sOR;
    this->sOS = sOS;
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
        pros::delay(0.5);
    }

    if (x != APS_NO_CHANGE)
        this->absX = x;

    if (y != APS_NO_CHANGE)
        this->absY = y;

    if (heading != APS_NO_CHANGE)
        this->absHeading = heading;

    positionDataMutex.give();
}

void APS::updateAbsolutePosition()
{
    /**
     * This function should be called at least once every 10 milliseconds for sufficient accuracy
     */
    int currLeftEncVal = this->leftEnc->get_value();
    int currRightEncVal = this->rightEnc->get_value();
    int currBackEncVal = this->strafeEnc->get_value();

    double dL = (currLeftEncVal - this->prevLeftEncVal) * this->wheelSize / 360.0;
    double dR = (currRightEncVal - this->prevRightEncVal) * this->wheelSize / 360.0;
    double dS = (currBackEncVal - this->prevStrafeEncVal) * this->wheelSize / 360.0;

    double oldHeading = this->absHeading;

    // calculated as an absolute quantity
    double newHeading = (180 / 3.141592) * (currLeftEncVal - currRightEncVal) / (this->sLO + this->sOR);

    // find dX, dY, dH from dL, dR, and dS
    // dY is the along the axis from the previous position to this position
    // dX is the along the axis perpendicular to that

    double dH = newHeading - oldHeading;
    double dX = dS / dH - sOS;
    double dY = dR / dH + sOR;

    dX *= 2 * sinDeg(dH / 2.0);
    dY *= 2 * sinDeg(dH / 2.0);

    // make sure nothing reads or writes from these variables
    // setting a few variables should never take more than one update cycle
    while (!this->positionDataMutex.take(5))
    {
        pros::delay(0.5);
    }

    this->absX += dX;
    this->absY += dY;
    this->absHeading = newHeading;

    this->positionDataMutex.give();

    this->prevLeftEncVal = currLeftEncVal;
    this->prevRightEncVal = currRightEncVal;
    this->prevStrafeEncVal = currBackEncVal;
}

absolutePosition APS::getAbsolutePosition()
{
    // make sure the variables aren't being written to while reading
    // I used a mutex instead of an atomic variable, as resetting the APS position will require a mutex, and I'm lazy
    while (!this->positionDataMutex.take(5))
    {
        pros::delay(0.5);
    }

    double x = this->absX;
    double y = this->absY;
    double h = this->absHeading;

    this->positionDataMutex.give();

    return {x, y, h};
}