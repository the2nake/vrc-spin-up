/**
 * Filename: APS.hpp
 * Author: home.vn2007@gmail.com
 * Copyright (c) 2023 by home.vn2007@gmail.com
 * All rights reserved
 */

#pragma once

#include "main.h"

#include <atomic>

#define APS_NO_CHANGE 32000

struct encoderConfig
{
    char topPort;
    char bottomPort;
    bool reversed = false;
};

struct absolutePosition {
    double x;
    double y;
    double heading;
};

struct odomConfig {
    double leftWheelOD; // in a two-encoder setup this is the y axis
    double rightWheelOD;
    double strafeWheelOD;
};

class APS
{
public:
    /**
     * Creates an APS object used to track position of the robot on the field, with the origin as its initialisation position
     * All size arguments are in inches
     *
     * @param leftEncoderConfig Configuration of the left encoder. Make sure that positive returned values indicate forward motion.
     * @param rightEncoderConfig Configuration of the right encoder. Make sure that positive returned values indicate forward motion.
     * @param strafeEncoderConfig Configuration of the back encoder. Make sure that positive returned values indicate rightward motion.
     *
     * @param sLO X position of the back encoder wheel, relative to the left encoder wheel, in millimetres;
     * @param sOR X position of the right encoder wheel, relative to the back encoder wheel, in millimetres;
     * @param sOS Y position of the back encoder wheel, relative to the left and right encoder wheels, in millimetres;
     * @param wheelODs Travel of tracking omniwheels, in millimitres.
     * 
     * @param imu An optional parameter to use an inertial measurement unit for the heading calculation
     * @param imuWeight How much weight to put into the IMU's measurements. From 0.0 to 1.0.
     */
    APS() {}
    APS(encoderConfig leftEncoderConfig, encoderConfig rightEncoderConfig, encoderConfig strafeEncoderConfig, double sLO, double sOR,
        double sOS, odomConfig wheelODs, pros::Imu* imu = nullptr, double imuWeight = 0.5);
    ~APS();

    /**
     * Sets the representation of the robot's current position and heading in the APS
    */
    virtual void setAbsolutePosition(double x = 0.0, double y = 0.0, double heading = 0.0);
    virtual void updateAbsolutePosition();
    virtual absolutePosition getAbsolutePosition();

private:
    double leftWheelOD = 220.0;
    double rightWheelOD = 220.0;
    double strafeWheelOD = 220.0;
    double sLO = 1.0, sOR = 1.0, sOS = 1.0;

    pros::ADIEncoder *leftEnc = nullptr;
    pros::ADIEncoder *rightEnc = nullptr;
    pros::ADIEncoder *strafeEnc = nullptr;

    std::atomic<double> absX = 0.0;
    std::atomic<double> absY = 0.0;
    std::atomic<double> absHeading = 0.0;

    pros::Mutex positionDataMutex;

    int prevLeftEncVal = 0.0;
    int prevRightEncVal = 0.0;
    int prevStrafeEncVal = 0.0;

    pros::Imu *imu = nullptr;
    double imuWeight = 0.0;

    bool encodersDisabled = false;
};