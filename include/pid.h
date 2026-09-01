#ifndef PID_CONTROL_H
#define PID_CONTROL_H


class PIDController {
public:
    PIDController();
    float compute(float setpoint, float measured_value, float dt);
    float clamp(float value, float out_min, float out_max);
    void setGains(float kp, float ki, float kd);
    void setLimits(float out_min, float out_max);

private:
    float kp_ = 0, ki_ = 0, kd_ = 0;
    float out_max_, out_min_;

    float integral_;
    float previous_error_;
};

#endif //PID_CONTROL_H
