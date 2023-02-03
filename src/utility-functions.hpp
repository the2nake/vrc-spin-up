#pragma once

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
