#ifndef POLOTICUS_ENCODER_H
#define POLOTICUS_ENCODER_H

#include <cstdint>


class Encoder {
    public:
        Encoder(uint8_t gpio, uint8_t ppr);
        ~Encoder();
        void initialize(uint32_t timer_window_ms, uint8_t pulse_window_intervals);
        void pulseCounterCallback();
        void encoderTimerWindowCallback();
        float determineTimeWindowAngularSpeed();
        float determinePulseWindowAngularSpeed();
        void reset();

    private:
        struct PulseCounter {
            volatile int32_t pulse_count;
            volatile uint64_t timestamp_us;
        }pulse_counter_;

        struct PulseWindow{
            volatile uint32_t pulse_count;
            volatile uint64_t time_interval_us;
        };
        
        PulseWindow pulse_window_; // Determine angular speed based on elapsed time between set pulse window
        PulseWindow timer_window_; // Determine angular speed based on pulse count within a time window

        const uint8_t gpio_, ppr_;

        uint32_t last_timer_window_pulse_count_;
        uint64_t last_pulse_window_timestamp_us_;
        static float determineAngularSpeed(const PulseWindow& pulse_window, uint8_t ppr);
};

#endif // POLOTICUS_ENCODER_H
