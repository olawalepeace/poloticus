#ifndef POLOTICUS_ENCODER_H
#define POLOTICUS_ENCODER_H

#include <cstdint>

struct PulseWindow{
    volatile uint32_t pulse_count = 0;
    volatile uint64_t time_interval_us = 0;
};


class Encoder {
    public:
        Encoder(uint8_t gpio, uint8_t ppr);
        ~Encoder();
        void initialize(uint32_t timer_window_ms, uint8_t pulse_window_intervals);
        float determineTimeWindowAngularSpeed();
        float determinePulseWindowAngularSpeed();
        void reset();

    private:
        struct PulseCounter {
            volatile int32_t pulse_count;
            volatile uint64_t timestamp_us;
        }pulse_counter_;
        
        PulseWindow pulse_window_; // Determine angular speed based on elapsed time between set pulse window
        PulseWindow timer_window_; // Determine angular speed based on pulse count within a time window

        const uint8_t gpio_, ppr_;

        uint32_t last_timer_window_pulse_count_;
        uint64_t last_pulse_window_timestamp_us_;
        void pulseCounterCallback_();
        void encoderTimerWindowCallback_();
        float determineAngularSpeed_(const PulseWindow& pulse_window);
};

#endif // POLOTICUS_ENCODER_H
