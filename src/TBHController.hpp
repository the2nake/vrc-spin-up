#pragma once

#include "main.h"

#include <atomic>
#include <chrono>

class TBHController
{
public:
    TBHController(double gain, double maximum = 0);

    void setActive(bool active) { this->active = active; }
    void setTarget(double target)
    {
        this->target = target;
        this->prevError = target;
    }
    void setGain(double gain) { this->gain = gain; }
    void setMaximum(double maximum) { this->maximum = maximum; }

    void setThresholds(double accuracy, double stability);

    void updateTBH(double sensor);
    void resetTBH();

    double getOutput() { return this->output.load(); }
    double getPrevError() { return this->prevError.load(); }
    bool isSettled() { return this->settled.load(); }
    void setSettled(bool settled) { this->settled = settled; }
    bool isActive() { return this->active.load(); }

private:
    std::atomic<bool> active = true;
    std::atomic<bool> settled = false;
    std::atomic<double> target = 0.0;
    std::atomic<double> maximum = 0.0;
    double gain = 0.0;
    std::atomic<double> prevError = 0.0;
    double tbh = 0.0;
    std::atomic<double> output = 0.0;

    std::chrono::high_resolution_clock::time_point lastUpdate;
    double accuracyThreshold = 1.0;
    double stabilityThreshold = 1.0;
};