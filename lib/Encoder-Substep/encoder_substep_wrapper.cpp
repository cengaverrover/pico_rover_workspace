#include "encoder_substep.hpp"

EncoderSubstep::EncoderSubstep(const PIO pio, const uint sm, const uint pinA) : m_pinA(pinA) {
    substep_init_state(pio, sm, m_pinA, &m_substepState);
    substep_set_calibration_data(&m_substepState, 64, 128, 192);
}

bool EncoderSubstep::calibrate() {

    substep_phases_t substep_phases = substep_calibrate_phases(m_substepState.pio, m_substepState.sm);
    if (substep_phases.first == 0) {
        substep_set_calibration_data(&m_substepState, 64, 128, 192);
        return false;
    } else {
        substep_set_calibration_data(&m_substepState, substep_phases.first, substep_phases.second, substep_phases.third);
        return true;
    }
}

uint EncoderSubstep::getStepCount() {
    substep_update(&m_substepState);
    return m_substepState.raw_step;
}

int EncoderSubstep::getSpeed() {
    substep_update(&m_substepState);
    return m_substepState.speed;
}

int EncoderSubstep::getSpeed_2_20() {
    substep_update(&m_substepState);
    return m_substepState.speed_2_20;
}

EncoderSubstep::Data EncoderSubstep::getAll() {
    substep_update(&m_substepState);
    Data data {};
    data.speed_2_20 = m_substepState.speed_2_20;
    data.speed = m_substepState.speed;
    data.step = m_substepState.raw_step;
    return data;
}