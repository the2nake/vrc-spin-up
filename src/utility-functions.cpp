/**
 * Filename: utility-functions.cpp
 * Author: home.vn2007@gmail.com
 * Copyright (c) 2023 by home.vn2007@gmail.com
 * All rights reserved
 */

#include "utility-functions.hpp"

#include <cmath>

#include "main.h"

/**
 * A function to calculate the modulo of a double, assuming b >= 0
 *
 * Returns a when b = 0
 */
double findMod(double a, double b)
{
    if (b == 0)
    {
        return a;
    }

    if (a < 0)
    {
        return findMod(a + b, b);
    }

    if (a >= b)
    {
        return findMod(a - b, b);
    }

    return a;
}

/**
 * Sine function using degrees
 */
double sinDeg(double deg)
{
    return std::sin(0.01745329251 * deg);
}

/**
 * Cosine function using degrees
 */
double cosDeg(double deg)
{
    return std::cos(0.01745329251 * deg);
}

/**
 * Arctangent function using degrees
 */

double atanDeg(double val)
{
    return 57.2957795131 * std::atan(val);
}

/**
 * Gets a pair of polar coordinates from a set of cartesian coordinates
 *
 * @param x Cartesian x-coordinate
 * @param y Cartesian y-coordinate
 *
 * @returns Returns a polarPoint
 */
polarPoint polarFromCartesian(double x, double y)
{
    polarPoint point{0, 0};

    point.rho = std::sqrt(x * x + y * y);

    if (x == 0)
    {
        if (y == 0)
        {
            point.theta = 0;
        }
        else
        {
            point.theta = 90 * (y > 0 ? 1 : -1);
        }
    }
    else
    {
        point.theta = findMod(360 + atanDeg(y / x) + (x < 0 ? 180 : 0), 360);
    }

    return point;
}

double rpmFromGearset(pros::motor_gearset_e_t gearing)
{
    switch (gearing)
    {
    case MOTOR_GEAR_100:
        return 100.0;
        break;
    case MOTOR_GEAR_200:
        return 200.0;
        break;
    case MOTOR_GEAR_600:
        return 600.0;
        break;
    default:
        return 0.0;
        break;
    }
}

double findShorterTurn(double a0, double af, double mod)
{
    auto rightAngle = findMod(af - a0, mod);
    if (std::abs(rightAngle) < std::abs(mod / 2.0))
    {
        return rightAngle;
    }
    else
    {
        return rightAngle - mod;
    }
}