#pragma once

#include <chrono>

struct PIDConfig {
    double fF;
    double kP;
    double kI;
    double kD;
    bool cutIntegral = false;
    double integralCutThreshold = 0.0;
};

class PIDController {
public:
    PIDController(PIDConfig config);

    void setPIDConstants(double fF, double kP, double kI, double kD, bool cutIntegral, double integralCutThreshold);
    void setPIDConstants(PIDConfig config);

    void startPID(double target);
    void setTarget(double target);
    void resetPIDSystem();
    double updatePID(double sensorValue);

    bool isActive() { return this->active; };
    
private:
    bool active = false;

    double target = 0.0;

    std::chrono::high_resolution_clock::time_point previousUpdateTimestamp;
    double fF = 0.0, kP = 0.0, kI = 0.0, kD = 0.0;
    double prevError = 0.0;
    double integral = 0.0;
    double derivative = 0.0;

    bool cutIntegral = false;
    double integralCutThreshold;
};
