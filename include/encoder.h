#ifndef POLOTICUS_ENCODER_H
#define POLOTICUS_ENCODER_H
#include <cstdint>

class Encoder {
    public:
        Encoder(uint8_t gpio, uint8_t ppr);
        ~Encoder();
        void initialize(uint32_t timer_window_ms, uint8_t pulse_window_intervals);
        void pulse_counter_callback();
        void encoder_timer_window_callback();
        float determine_time_window_angular_speed();
        float determine_pulse_window_angular_speed(uint64_t timestamp_us);
        // float get_time_window_angular_speed() { return time_window_angular_speed; }
        // float get_pulse_window_angular_speed() { return pulse_window_angular_speed; }
        void reset();

    private:
        struct pulse_counter {
            int32_t pulse_count;
            uint64_t timestamp_us;
        }pulse_counter;

        uint8_t gpio_, ppr_;

        uint8_t pulse_window_intervals;
        uint32_t timer_window_ms;
        float pulse_window_angular_speed, time_window_angular_speed;

        uint32_t last_timer_window_pulse_count;
        uint64_t last_pulse_window_timestamp_us;
        float determine_angular_speed(uint64_t pulse_count_diff, uint64_t time_diff_us);
};

#endif // POLOTICUS_ENCODER_H
