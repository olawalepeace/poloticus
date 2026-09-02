#ifndef WHEEL_H
#define WHEEL_H

#include "encoder.h"
#include "motor.h"
#include "pid.h"

class Wheel{
public:
    Wheel(Motor* motor, Encoder* Encoder = nullptr);
    void commandVelocity(float velociy); // velocity range: -1.0 to 1.0
    float getVelocity(); // Returns the current velocity of the wheel in the range of -1.0 to 1.0
    void tunePIDGains(float kp, float ki, float kd);
    void stop();

private:
    Encoder* encoder_;
    Motor* motor_;
    PIDController pid_controller_;
    float velocity_ = 0;
    bool it_is_using_PID_ = false;
};

#endif //WHEEL_H
