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
    this->timer_window_ms = timer_window_ms;
    this->pulse_window_intervals = pulse_window_intervals;
    // Here, I will create the two interrupts
    reset();
}

void Encoder::pulse_counter_callback() {
    // Callback function to handle pulse counting
    pulse_counter.pulse_count++;
    pulse_counter.timestamp_us = to_us_since_boot(get_absolute_time());
    if (pulse_counter.pulse_count == 1) {
        last_pulse_window_timestamp_us = pulse_counter.timestamp_us;
        return; // Skip speed calculation for the first pulse, if not, it'll calculate a speed of 0, harmless but not necessary
    }
    if ((pulse_counter.pulse_count-1) % pulse_window_intervals == 0) { //subtracting 1 because we start counting from 1
        pulse_window_angular_speed = determine_pulse_window_angular_speed(pulse_counter.timestamp_us);
    }
}

void Encoder::encoder_timer_window_callback() {
    // Callback function to handle timer window events
    time_window_angular_speed = determine_time_window_angular_speed();
}

void Encoder::reset() {
    pulse_counter.pulse_count = 0;
    pulse_counter.timestamp_us = 0;
    last_timer_window_pulse_count = 0;
    last_pulse_window_timestamp_us = 0;
}

float Encoder::determine_time_window_angular_speed() {
    // Implementation for determining angular speed with time window
    uint32_t pulse_count_diff = pulse_counter.pulse_count - last_timer_window_pulse_count;
    last_timer_window_pulse_count = pulse_counter.pulse_count;    
    return determine_angular_speed(pulse_count_diff, timer_window_ms * 1000); // Convert ms to us   
}

float Encoder::determine_pulse_window_angular_speed(uint64_t timestamp_us) {
    // Implementation for determining angular speed with pulse window
    uint64_t time_diff_us = (timestamp_us - last_pulse_window_timestamp_us);
    last_pulse_window_timestamp_us = timestamp_us;
    return determine_angular_speed(pulse_window_intervals, time_diff_us);
}

float Encoder::determine_angular_speed(uint64_t pulse_count_diff, uint64_t time_diff_us) {
    // Calculate angular speed based on pulse count difference and time difference
    if (time_diff_us == 0) {
        return 0.0f; // Avoid division by zero
    }
    float revolutions = (float)pulse_count_diff / ppr_;
    float time_diff_s = (float)time_diff_us / 1000000.0f; // Convert microseconds to seconds
    float angular_speed = revolutions / time_diff_s; // in revolutions per second
    return angular_speed * 2.0f * 3.14159265358979323846f; // Convert to radians per second
}