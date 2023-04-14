#include "TBHController.hpp"

#include "utility-functions.hpp"

TBHController::TBHController(double gain)
{
    this->lastUpdate = std::chrono::high_resolution_clock::now();
    this->gain = gain;
}

void TBHController::setThresholds(double acc, double stab)
{
    this->accuracyThreshold = acc;
    this->stabilityThreshold = stab;
}

void TBHController::updateTBH(double sensor)
{
    auto now = std::chrono::high_resolution_clock::now();

    if (!this->active.load())
    {
        this->lastUpdate = now;
        return;
    }

    double error = this->target - sensor;
    auto dT = timeBetween<std::chrono::milliseconds>(this->lastUpdate, now) / 1000.0;
    if (dT > 1.0)
    {
        this->lastUpdate = now;
        return;
    }

    this->output = this->output + (this->gain * error) * dT;
    if (std::signbit(error) != std::signbit(this->prevError))
    {
        this->output = 0.5 * (this->output + this->tbh);
        this->tbh = this->output;
        this->prevError = error;
    }
    this->settled = std::abs(error) <= accuracyThreshold && std::abs(error - this->prevError) <= stabilityThreshold;

    pros::screen::print(TEXT_MEDIUM, 1, "Input: %f Output: %f Integral: %f dT: %f", sensor, this->output.load(), this->tbh, dT);

    this->lastUpdate = now;
}

void TBHController::resetTBH()
{
    this->settled = false;
    this->tbh = 0.0;
    this->prevError = 0.0;
}