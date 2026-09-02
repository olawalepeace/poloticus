#include "pid.h"

#include "pico/time.h"


PIDController::PIDController(){}

void PIDController::setGains(float kp, float ki, float kd)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    reset();
}

void PIDController::setLimits(float out_min, float out_max)
{
    out_min_ = out_min;
    out_max_ = out_max;
    reset();
}

float PIDController::compute(float setpoint, float measured_value)
{
    uint64_t current_time = to_us_since_boot(get_absolute_time());
    float output = 0.0f;
    if(previous_time_ > 0){
        float dt = (current_time-previous_time_)/1e6f;
        float error = setpoint - measured_value;

        integral_ += error * dt;
        // clamp(integral_, cfg_.integral_min, cfg_.integral_max);

        float derivative = (dt > 0.0f) ? (error - previous_error_) / dt : 0.0f;
        previous_error_ = error;

        output = kp_ * error + ki_ * integral_ + kd_ * derivative;
        clamp(output, out_min_, out_max_);
    }
    previous_time_ = current_time;
    return output;
}

void PIDController::reset()
{
    integral_ = 0.0f;
    previous_error_ = 0.0f;
    previous_time_ = 0;
}

void PIDController::clamp(float& value, float minimum,float maximum)
{
    if (value > maximum) value = maximum;
    if (value <minimum) value = minimum;
}
