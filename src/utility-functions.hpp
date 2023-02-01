#pragma once

/**
 * A function to calculate the modulo of a double
 */
double findMod(double a, double b);

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
 * Returns a polarPoint structure
 */
polarPoint polarFromCartesian(double x, double y);