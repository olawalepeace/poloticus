#include "buzzer.h"

#include "pico/stdlib.h"


Buzzer::Buzzer(uint8_t gpio): buzzer_pin_(gpio)
{
    gpio_init(buzzer_pin_);
    gpio_set_dir(buzzer_pin_, GPIO_OUT);
}

/**
 * Produces a single blocking buzzer pulse at the provided GPIO pin.
 * 
 * `Warning`: this is blocking and not recommended in main loop.
 * @param gpio The GPIO pin that should be driven high for the pulse.
 * @param duration The pulse length in milliseconds.
 */
void Buzzer::beep(uint8_t gpio, uint16_t duration)
{
    gpio_init(gpio);
    gpio_set_dir(gpio, GPIO_OUT);
    gpio_put(gpio, 1);
    // TODO: There will be a unit-duration rest here, will figure out a non-blocking approach
    sleep_ms(duration);
    
    gpio_put(gpio, 0);
    gpio_disable_pulls(gpio);
}

/**
 * Schedules a repeating buzzer pattern that runs for a fixed total duration.
 *
 * @param on_period The active pulse length in milliseconds.
 * @param off_period The silent interval length in milliseconds between pulses.
 * @param duration The total time the buzzer should continue sounding, in milliseconds.
 */
void Buzzer::timedBeep(uint16_t on_period, uint16_t off_period, uint16_t duration)
{
    BuzzerDuty new_duty;
    new_duty.is_beeping= true;
    new_duty.start_time = new_duty.next_toggle = to_us_since_boot(get_absolute_time());
    new_duty.termination_type = TerminationType::TIME;
    new_duty.on_period = on_period*1000;
    new_duty.off_period = off_period*1000;
    new_duty.end.time = new_duty.start_time + duration*1000000;

    setDuty_(new_duty);
}

/**
 * Schedules a repeating buzzer pattern that ends after a set number of on/off cycles.
 *
 * @param on_period The active pulse length in milliseconds.
 * @param off_period The silent interval length in milliseconds between pulses.
 * @param beep_count The number of beeps to complete before stopping.
 */
void Buzzer::countBeep(uint16_t on_period, uint16_t off_period, uint16_t beep_count)
{
    BuzzerDuty new_duty;
    new_duty.is_beeping= true;
    new_duty.start_time = new_duty.next_toggle = to_us_since_boot(get_absolute_time());
    new_duty.termination_type = TerminationType::COUNT;
    new_duty.on_period = on_period*1000;
    new_duty.off_period = off_period*1000;
    new_duty.end.count = beep_count;

    setDuty_(new_duty);
}

/**
 * Advances the current buzzer pattern and toggles the output on the scheduled timing boundary.
 *
 * If the configured pattern is time-based or count-based, this routine stops the buzzer once the
 * termination condition is reached.
 */
void Buzzer::update()
{
    if (!buzzer_duty_.is_beeping){return;}
    uint64_t time_now = to_us_since_boot(get_absolute_time());
    if(time_now >= buzzer_duty_.next_toggle){
        buzzer_duty_.high = !buzzer_duty_.high;
        gpio_put(buzzer_pin_, (uint8_t)(buzzer_duty_.high));
        if (buzzer_duty_.high){
            buzzer_duty_.next_toggle += buzzer_duty_.on_period;
        }else{
            buzzer_duty_.next_toggle += buzzer_duty_.off_period;
            buzzer_duty_.count += 1;
        }
    }
    if (
        ((buzzer_duty_.termination_type == TerminationType::COUNT) &&
            (buzzer_duty_.count == buzzer_duty_.end.count)) ||
        ((buzzer_duty_.termination_type == TerminationType::TIME) &&
            (time_now >= buzzer_duty_.end.time))
        ){
        reset();
    }    
}

void Buzzer::reset(){
    gpio_put(buzzer_pin_, 0);
    BuzzerDuty reset_duty;
    setDuty_(reset_duty);
}

/**
 * Stores a new buzzer duty configuration as the active behavior for the next update cycle.
 *
 * @param duty (`BuzzerDuty`)The duty schedule that should be applied to the buzzer state machine.
 */
void Buzzer::setDuty_(BuzzerDuty duty)
{
    buzzer_duty_ = duty;
}
