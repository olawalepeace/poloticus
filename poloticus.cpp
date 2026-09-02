#include <stdio.h>

#include "mpu6050.h"
#include "motor.h"
#include "wheel.h"

#include "pico/stdlib.h"

#define CONTROL_LOOP_FREQ 50
#define CONTROL_LOOP_PERIOD_US (1000000 / CONTROL_LOOP_FREQ)

Motor right_motor(13, 11, 12);
Motor left_motor(18, 19, 20);
Wheel right_wheel(&right_motor);
Wheel left_wheel(&left_motor);

uint64_t get_time_difference(uint64_t previous_time)
{
    return (to_us_since_boot(get_absolute_time())- previous_time);
}

void control_loop_callback(){
    left_wheel.commandVelocity(0.1);
    right_wheel.commandVelocity(0.1);
} // example usage

int main() {
    stdio_init_all();
    right_motor.initialize(4999, 1.0, 3800);
    left_motor.initialize(4999, 1.0, 3800);

    uint64_t last_call_time = to_us_since_boot(get_absolute_time());
    while (true) {
        if (get_time_difference(last_call_time) >= CONTROL_LOOP_PERIOD_US) {
            control_loop_callback();
            last_call_time += CONTROL_LOOP_PERIOD_US;
        }  
    }
}
