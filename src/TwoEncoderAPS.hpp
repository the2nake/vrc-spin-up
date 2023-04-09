/**
 * Filename: TwoEncoderAPS.hpp
 * Author: home.vn2007@gmail.com
 * Copyright (c) 2023 by home.vn2007@gmail.com
 * All rights reserved
 */

#pragma once

#include "APS.hpp"

#include "main.h"

#include <atomic>

struct EncoderReadingPair {
    double x;
    double y;
};

class TwoEncoderAPS : public APS {
public:
    /**
     * sYO is measured as distance to the right of the origin
     * SOX is measured as distance above the origin
    */
    TwoEncoderAPS(encoderConfig yEncoderConfig, encoderConfig xEncoderConfig, double sYO, double sOX, odomConfig wheelODs, pros::Imu* imu, double imuMultiplier = 1.0);
    ~TwoEncoderAPS();

    /**
     * Sets the representation of the robot's current position and heading in the APS
    */
    void setAbsolutePosition(double x = 0.0, double y = 0.0, double heading = 0.0) override;
    void updateAbsolutePosition() override;
    absolutePosition getAbsolutePosition() override;
    EncoderReadingPair getEncoderReadings() {
        return {(double)(prevXEncVal), (double)(prevYEncVal)};
    }

private:
    double yWheelOD = 220.0; // in mm
    double xWheelOD = 220.0; // in mm
    double sYO = 1.0, sOX = 1.0;

    pros::ADIEncoder *yEnc = nullptr;
    pros::ADIEncoder *xEnc = nullptr;

    std::atomic<double> absX = 0.0;
    std::atomic<double> absY = 0.0;
    std::atomic<double> absHeading = 0.0;

    pros::Mutex positionDataMutex;

    int prevYEncVal = 0;
    int prevXEncVal = 0;
    double prevHeading = 0.01;

    pros::Imu *imu = nullptr;
    double previousImuHeading = 0.0;
    double imuMultiplier = 1.0;

    bool encodersDisabled = false;
};