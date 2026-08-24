#include "encoder.h"

#include "pico/stdlib.h"
#include "pico/time.h"

Encoder::Encoder(uint8_t gpio, uint8_t ppr): gpio_(gpio), ppr_(ppr)
{
}

Encoder::~Encoder() {
}

void Encoder::initialize(uint32_t timer_window_ms, uint8_t pulse_window_intervals) {
    // Initialization code for the encoder
    timer_window_.time_interval_us = timer_window_ms*1000;
    pulse_window_.pulse_count = pulse_window_intervals;
    // Here, I will create the two interrupts
    reset();
}

void Encoder::pulse_counter_callback() {
    // Callback function to handle pulse counting
    pulse_counter_.pulse_count++;
    pulse_counter_.timestamp_us = to_us_since_boot(get_absolute_time());
    if (pulse_counter_.pulse_count == 1) {
        last_pulse_window_timestamp_us_ = pulse_counter_.timestamp_us;
        return; // Skip speed calculation for the first pulse, if not, it'll calculate a speed of 0, harmless but not necessary
    }
    if ((pulse_counter_.pulse_count-1) % pulse_window_.pulse_count == 0) { //subtracting 1 because we start counting from 1
        pulse_window_.time_interval_us = pulse_counter_.timestamp_us - last_pulse_window_timestamp_us_;
        last_pulse_window_timestamp_us_ = pulse_counter_.timestamp_us;
    }
}

void Encoder::encoder_timer_window_callback() {
    // Callback function to handle timer window events
    timer_window_.pulse_count = pulse_counter_.pulse_count - last_timer_window_pulse_count_;
    last_timer_window_pulse_count_ = pulse_counter_.pulse_count; 
}

void Encoder::reset() {
    pulse_counter_.pulse_count = 0;
    pulse_counter_.timestamp_us = 0;
    last_timer_window_pulse_count_ = 0;
    last_pulse_window_timestamp_us_ = 0;
}

float Encoder::determine_angular_speed(const Encoder::PulseWindow& pulse_window, uint8_t ppr) {
    // Calculate angular speed based on pulse count difference and time difference
    if (pulse_window.time_interval_us == 0 or ppr == 0) {
        return 0.0f; // Avoid division by zero
    }
    float revolutions = (float)pulse_window.pulse_count / ppr; // Calculate revolutions based on pulse count and pulses per revolution
    float time_diff_s = (float)pulse_window.time_interval_us / 1000000.0f; // Convert microseconds to seconds
    float angular_speed = revolutions / time_diff_s; // in revolutions per second
    return angular_speed * 2.0f * 3.14159265358979323846f; // Convert to radians per second
}
