/**
 * Filename: utility-functions.hpp
 * Author: home.vn2007@gmail.com
 * Copyright (c) 2023 by home.vn2007@gmail.com
 * All rights reserved
 */

#pragma once

#include "main.h"

#include <chrono>

/**
 * A function to calculate the modulo of a double
 */
double findMod(double a, double b);

/**
 * Sine function using degrees
 */
double sinDeg(double deg);

/**
 * Cosine function using degrees
 */
double cosDeg(double deg);

/**
 * Arctangent function using degrees
 */
double atanDeg(double val);

struct polarPoint
{
    double rho = 0;
    double theta = 0;
};

/**
 * Gets a pair of polar coordinates from a set of cartesian coordinates
 *
 * @param x Cartesian x-coordinate
 * @param y Cartesian y-coordinate
 *
 * @returns Returns a polarPoint
 */
polarPoint polarFromCartesian(double x, double y);

double rpmFromGearset(pros::motor_gearset_e_t gearing);

double findShorterTurn(double a0, double af, double mod);

double inDegrees(double radians);

double inRadians(double degrees);

template <typename T>
T vectorMax(std::vector<T> &vector)
{
    T max = vector.front();
    for (T item : vector)
    {
        if (item > max)
        {
            max = item;
        }
    }
    return max;
}

template <typename T>
long double timeBetween(std::chrono::high_resolution_clock::time_point t0, std::chrono::high_resolution_clock::time_point tf) {
    return std::chrono::duration_cast<T>(tf - t0).count();
}
