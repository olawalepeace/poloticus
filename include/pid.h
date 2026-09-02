#ifndef PID_CONTROL_H
#define PID_CONTROL_H

#include <cstdint>


class PIDController {
public:
    PIDController();
    float compute(float setpoint, float measured_value);
    void clamp(float& value, float minimum, float maximum);
    void setGains(float kp, float ki, float kd);

    //will use this onl during tuning; beyond that, calls to this is not adviced
    void setKp(float kp) { kp_ = kp; }
    void setKi(float ki) { ki_ = ki; }
    void setKd(float kd) { kd_ = kd; }

    void setLimits(float out_min, float out_max);
    void reset();

private:
    float kp_ = 0, ki_ = 0, kd_ = 0;
    float out_max_, out_min_;

    float integral_;
    float previous_error_;
    uint64_t previous_time_;
};

#endif //PID_CONTROL_H
