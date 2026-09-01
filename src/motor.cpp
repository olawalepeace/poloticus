#include "motor.h"

#include <cmath>
#include "hardware/gpio.h"
#include "hardware/pwm.h"


Motor::Motor(uint8_t pwm_pin, uint8_t dir_pin_forward, uint8_t dir_pin_reverse, float norm_vel_max):
    pwm_pin_(pwm_pin), dir_pin_forward_(dir_pin_forward), dir_pin_reverse_(dir_pin_reverse), norm_vel_max_(norm_vel_max) {}

void Motor::initialize()
{
    gpio_init(dir_pin_forward_);
    gpio_set_dir(dir_pin_forward_, GPIO_OUT);
    gpio_init(dir_pin_reverse_);
    gpio_set_dir(dir_pin_reverse_, GPIO_OUT);
    
    gpio_set_function(pwm_pin_, GPIO_FUNC_PWM);
    wrap_ = pwm_get_default_config().top;
    pwm_set_enabled(pwm_gpio_to_slice_num(pwm_pin_), true);
}

void Motor::initialize(uint16_t wrap, float clock_div, uint16_t min_wrap)
{
    initialize();
    uint16_t slice_num = pwm_gpio_to_slice_num(pwm_pin_);
    pwm_set_clkdiv(slice_num, clock_div);
    pwm_set_wrap(slice_num, wrap);
    wrap_min_ = min_wrap;
    wrap_ = wrap;
    stop();
}

void Motor::commandVelocity(float velocity)
{
    if (velocity>0){
        commandDirection_(1,0);
        setDirection_(MotorDirection::FORWARD);
    }
    else if (velocity<0)
    {
        commandDirection_(0,1);
        setDirection_(MotorDirection::REVERSE);
    }
    else{
        commandDirection_(0, 0);
    }
    velocity = std::fabs(velocity);
    uint32_t pwm_level = map(velocity);
    pwm_set_gpio_level(pwm_pin_, pwm_level);
}

void Motor::commandDirection_(uint8_t forward, uint8_t reverse)
{
    gpio_put(dir_pin_forward_, forward);
    gpio_put(dir_pin_reverse_, reverse);
}

void Motor::stop(){
    commandDirection_(1,1);
    setDirection_(MotorDirection::BRAKE);
}

void Motor::setDirection_(MotorDirection motor_direction)
{
    motor_direction_ = motor_direction;
}

MotorDirection Motor::getDirection()
{  
    return motor_direction_;
}

uint32_t Motor::map(float vel)
{
    if (vel == 0){
        return 0;
    }
    if (vel >= norm_vel_max_){
        return wrap_;
    }
   return (vel/norm_vel_max_) * (wrap_ - wrap_min_)  + wrap_min_;
}
