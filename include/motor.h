#ifndef MOTOR_H
#define MOTOR_H

#include <cstdint>

enum MotorDirection {
    FORWARD,
    REVERSE,
    BRAKE
};


class Motor {
public:
    Motor(uint8_t pwm_pin, uint8_t dir_pin_forward, uint8_t dir_pin_reverse, float norm_vel_max = 12.0);
    ~Motor();

    void initialize();
    void initialize(uint16_t wrap, float clock_div);
    void commandVelocity(float velocity);
    void stop(); // stops the motor and prevents any motion at all.
    MotorDirection getDirection();

private:
    const uint8_t pwm_pin_;
    const uint8_t dir_pin_forward_, dir_pin_reverse_;
    uint16_t wrap_;
    uint16_t wrap_min_;
    const float norm_vel_max_;
    MotorDirection motor_direction_;
    
    uint32_t map(float value);
    void setWrapMin_(uint16_t wrap_min); //TODO: I have to find a new name for this, this doesn't just look right
    void setDirection_(MotorDirection motor_direction);
    void commandDirection_(uint8_t forward, uint8_t reverse);
};

#endif //MOTOR_H
