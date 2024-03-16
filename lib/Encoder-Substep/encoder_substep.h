#ifndef __ENCODER_SUBSTEP_H__
#define __ENCODER_SUBSTEP_H__

#ifdef __cplusplus
extern "C" {
#endif


#include "pico/stdlib.h"
#include "hardware/pio.h"

#include "encoder_substep.pio.h"


typedef struct substep_state_t {
    // configuration data:
    uint calibration_data[4]; // relative phase sizes
    uint clocks_per_us;       // save the clk_sys frequency in clocks per us
    uint idle_stop_samples;   // after these samples without transitions, assume the encoder is stopped
    PIO pio;
    uint sm;

    // internal fields to keep track of the previous state:
    uint prev_trans_pos, prev_trans_us;
    uint prev_step_us;
    uint prev_low, prev_high;
    uint idle_stop_sample_count;
    int speed_2_20;
    int stopped;

    // output of the encoder update function:
    int speed;     // estimated speed in substeps per second
    uint position; // estimated position in substeps

    uint raw_step; // raw step count
} substep_state_t;

typedef struct substep_phases_t {
    int first;
    int second;
    int third;
} substep_phases_t;

uint get_step_start_transition_pos(substep_state_t* state, uint step);

int substep_calc_speed(int delta_substep, int delta_us);

void substep_init_state(PIO pio, int sm, int pin_a, substep_state_t* state);

void substep_update(substep_state_t* state);

void substep_init_state(PIO pio, int sm, int pin_a, substep_state_t* state);

substep_phases_t substep_calibrate_phases(PIO pio, uint sm);

void substep_set_calibration_data(substep_state_t* state, int step0, int step1, int step2);

#ifdef __cplusplus
}
#endif


#endif 
