#ifndef POLOTICUS_BUZZER
#define POLOTICUS_BUZZER

#include <cstdint>

enum class TerminationType {
    COUNT,
    TIME,
};

struct BuzzerDuty
{
    bool is_beeping = false;
    uint64_t start_time;
    TerminationType termination_type;
    uint16_t count = 0;
    uint64_t next_toggle;
    uint64_t on_period;
    uint64_t off_period;
    bool high = false;
    union
    {
        uint64_t time;
        uint16_t count;
    }end;
};


class Buzzer
{
public:
    Buzzer(uint8_t gpio);
    ~Buzzer();

    void timedBeep(uint16_t on_period, uint16_t off_period, uint16_t duration);
    void countBeep(uint16_t on_period, uint16_t off_period, uint16_t beep_count);
    void update();
    static void beep(uint8_t gpio, uint16_t duration);
    void reset();
    
private:
    const uint8_t buzzer_pin_;
    BuzzerDuty buzzer_duty_;
    void setDuty_(BuzzerDuty duty);
};

#endif