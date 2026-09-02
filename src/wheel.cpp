#include "wheel.h"


Wheel::Wheel (Motor* motor, Encoder* encoder): motor_(motor), encoder_(encoder)
{
    float motor_norm_velocity_max = motor_->getNormVelocity();
    pid_controller_.setLimits(-motor_norm_velocity_max, motor_norm_velocity_max);
};

void Wheel::tunePIDGains(float kp, float ki, float kd)
{
    if (encoder_ == nullptr) return;
    pid_controller_.setGains(kp, ki, kd);
    it_is_using_PID_ = true;
}

void Wheel::commandVelocity(float velocity)
{
    float control = velocity;
    if(encoder_ == nullptr && it_is_using_PID_){
        control = pid_controller_.compute(velocity, velocity);
    }
    else if (it_is_using_PID_){
        float current_velocity = getVelocity();
        control = pid_controller_.compute(velocity, current_velocity);
    }
    motor_->commandVelocity(control);
    velocity_ = velocity;
}

float Wheel::getVelocity()
{
    if(encoder_ == nullptr) return 0.0;
    if(velocity_ >= 7) return encoder_->determinePulseWindowAngularSpeed();
    if(velocity_ < 7) return encoder_->determineTimeWindowAngularSpeed();

    //TODO: will maybe add a fusion alg here, will run some tests and conclude
    return 0.0;
}

void Wheel::stop()
{
    motor_->stop();
}
