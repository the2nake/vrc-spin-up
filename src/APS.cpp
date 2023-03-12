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
         double sLO, double sOR, double sOS, odomConfig wheelSizes)
{
    this->leftEnc = new pros::ADIEncoder(leftEncoderConfig.topPort, leftEncoderConfig.bottomPort, leftEncoderConfig.reversed);
    this->rightEnc = new pros::ADIEncoder(rightEncoderConfig.topPort, rightEncoderConfig.bottomPort, rightEncoderConfig.reversed);
    this->strafeEnc = new pros::ADIEncoder(strafeEncoderConfig.topPort, strafeEncoderConfig.bottomPort, strafeEncoderConfig.reversed);

    this->leftWheelSize = wheelSizes.leftWheelSize * 3.141592;
    this->rightWheelSize = wheelSizes.rightWheelSize * 3.141592;
    this->strafeWheelSize = wheelSizes.strafeWheelSize * 3.141592;
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
    }

    this->leftEnc->reset();
    this->rightEnc->reset();
    this->strafeEnc->reset();

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
    }

    positionDataMutex.give();
}

void APS::updateAbsolutePosition()
{
    /**
     * This function should be called at least once every 10 milliseconds for sufficient accuracy
     */
    int currLeftEncVal = this->leftEnc->get_value();
    int currRightEncVal = this->rightEnc->get_value();
    int currStrafeEncVal = this->strafeEnc->get_value();

    double dL = (currLeftEncVal - this->prevLeftEncVal) * this->leftWheelSize / 360.0;
    double dR = (currRightEncVal - this->prevRightEncVal) * this->rightWheelSize / 360.0;
    double dS = (currStrafeEncVal - this->prevStrafeEncVal) * this->strafeWheelSize / 360.0;

    double oldHeading = this->absHeading;

    // calculated as an absolute quantity
    double newHeading = (180 / 3.141592) * ((currLeftEncVal / 360.0) * this->leftWheelSize - (currRightEncVal / 360.0) * this->rightWheelSize) / (this->sLO + this->sOR);

    // find dX, dY, dH from dL, dR, and dS
    // dY is the along the axis from the previous position to this position
    // dX is the along the axis perpendicular to that

    double dH = newHeading - oldHeading;
    double dX = dS / dH - sOS;
    double dY = (dR / dH + sOR + dL / dH + sLO) / 2.0;

    dX *= 2 * sinDeg(dH / 2.0);
    dY *= 2 * sinDeg(dH / 2.0);

    // make sure nothing reads or writes from these variables
    // setting a few variables should never take more than one update cycle
    while (!this->positionDataMutex.take(5))
    {
    }
    this->absX = this->absX + dX;
    this->absY = this->absY + dY;
    this->absHeading = this->absHeading + dH;
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