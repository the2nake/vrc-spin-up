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
 * A function to calculate the modulo of a double
 */
double findMod(double a, double b)
{
    double mod;
    // Handling negative values
    if (a < 0)
        mod = -a;
    else
        mod = a;
    if (b < 0)
        b = -b;

    // Finding mod by repeated subtraction

    while (mod >= b)
        mod = mod - b;

    // Sign of result typically depends
    // on sign of a.
    if (a < 0)
        return -mod;

    return mod;
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
