#ifndef __ENCODER_SUBSTEP_HPP__
#define __ENCODER_SUBSTEP_HPP__

#include "encoder_substep.h"

class EncoderSubstep {
public:
    EncoderSubstep() = delete;
    EncoderSubstep(EncoderSubstep& encoder) = delete;

    EncoderSubstep(const PIO pio, const uint sm, const uint pinA);

    bool calibrate();

    uint getStepCount();

    int getSpeed();

    int getSpeed_2_20();

    struct Data {
        int speed_2_20;
        int speed;
        uint step;
    };

    Data getAll();

private:
    const uint m_pinA {};
    substep_state_t m_substepState {};

};

#endif // __ENCODER_SUBSTEP_HPP__