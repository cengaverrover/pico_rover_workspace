
#include <stdio.h>
#include <string.h>
#include "hardware/pio.h"
#include "hardware/timer.h"
#include "hardware/pwm.h"
#include <pico/divider.h>

#include "encoder_substep.h"

// internal helper functions (not to be used by user code)

void read_pio_data(substep_state_t *state, uint *step, uint *step_us, uint *transition_us, int *forward)
{
    int cycles;

    // get the raw data from the PIO state machine
    quadrature_encoder_substep_get_counts(state->pio, state->sm, step, &cycles, step_us);

    // when the PIO program detects a transition, it sets cycles to either zero
    // (when step is incrementing) or 2^31 (when step is decrementing) and keeps
    // decrementing it on each 13 clock loop. We can use this information to get
    // the time and direction of the last transition
    if (cycles < 0) {
        cycles = -cycles;
        *forward = 1;
    } else {
        cycles = 0x80000000 - cycles;
        *forward = 0;
    }
    *transition_us = *step_us - ((cycles * 13) / state->clocks_per_us);
}

// get the sub-step position of the start of a step
uint get_step_start_transition_pos(substep_state_t *state, uint step)
{
    return ((step << 6) & 0xFFFFFF00) | state->calibration_data[step & 3];
}

// compute speed in "sub-steps per 2^20 us" from a delta substep position and
// delta time in microseconds. This unit is cheaper to compute and use, so we
// only convert to "sub-steps per second" once per update, at most
int substep_calc_speed(int delta_substep, int delta_us)
{
    return ((int64_t) delta_substep << 20) / delta_us;
}



// main functions to be used by user code

// initialize the substep state structure and start PIO code
void substep_init_state(PIO pio, int sm, int pin_a, substep_state_t *state)
{
    int forward;

    // set all fields to zero by default
    memset(state, 0, sizeof(substep_state_t));

    // initialize the PIO program (and save the PIO reference)
    state->pio = pio;
    state->sm = sm;
    quadrature_encoder_substep_program_init(pio, sm, pin_a);

    // start with equal phase size calibration
    state->calibration_data[0] = 0;
    state->calibration_data[1] = 64;
    state->calibration_data[2] = 128;
    state->calibration_data[3] = 192;

    state->idle_stop_samples = 3;

    // start "stopped" so that we don't use stale data to compute speeds
    state->stopped = 1;

    // cache the PIO cycles per us
    state->clocks_per_us = (clock_get_hz(clk_sys) + 500000) / 1000000;

    // initialize the "previous state"
    read_pio_data(state, &state->raw_step, &state->prev_step_us, &state->prev_trans_us, &forward);

    state->position = get_step_start_transition_pos(state, state->raw_step) + 32;
}


// read the PIO data and update the speed / position estimate
void substep_update(substep_state_t *state)
{
    uint step, step_us, transition_us, transition_pos, low, high;
    int forward, speed_high, speed_low;

    // read the current encoder state from the PIO
    read_pio_data(state, &step, &step_us, &transition_us, &forward);

    // from the current step we can get the low and high boundaries in substeps
    // of the current position
    low = get_step_start_transition_pos(state, step);
    high = get_step_start_transition_pos(state, step + 1);

    // if we were not stopped, but the last transition was more than
    // "idle_stop_samples" ago, we are stopped now
    if (step == state->raw_step)
        state->idle_stop_sample_count++;
    else
        state->idle_stop_sample_count = 0;

    if (!state->stopped && state->idle_stop_sample_count >= state->idle_stop_samples) {
        state->speed = 0;
        state->speed_2_20 = 0;
        state->stopped = 1;
    }

    // when we are at a different step now, there is certainly a transition
    if (state->raw_step != step) {
        // the transition position depends on the direction of the move
        transition_pos = forward ? low : high;

        // if we are not stopped, that means there is valid previous transition
        // we can use to estimate the current speed
        if (!state->stopped)
            state->speed_2_20 = substep_calc_speed(transition_pos - state->prev_trans_pos, transition_us - state->prev_trans_us);

        // if we have a transition, we are not stopped now
        state->stopped = 0;
        // save the timestamp and position of this transition to use later to
        // estimate speed
        state->prev_trans_pos = transition_pos;
        state->prev_trans_us = transition_us;
    }

    // if we are stopped, speed is zero and the position estimate remains
    // constant. If we are not stopped, we have to update the position and speed
    if (!state->stopped) {
        // although the current step doesn't give us a precise position, it does
        // give boundaries to the position, which together with the last
        // transition gives us boundaries for the speed value. This can be very
        // useful especially in two situations:
        // - we have been stopped for a while and start moving quickly: although
        //   we only have one transition initially, the number of steps we moved
        //   can already give a non-zero speed estimate
        // - we were moving but then stop: without any extra logic we would just
        //   keep the last speed for a while, but we know from the step
        //   boundaries that the speed must be decreasing

        // if there is a transition between the last sample and now, and that
        // transition is closer to now than the previous sample time, we should
        // use the slopes from the last sample to the transition as these will
        // have less numerical issues and produce a tighter boundary
        if (state->prev_trans_us > state->prev_step_us && 
            (int)(state->prev_trans_us - state->prev_step_us) > (int)(step_us - state->prev_trans_us)) {
            speed_high = substep_calc_speed(state->prev_trans_pos - state->prev_low, state->prev_trans_us - state->prev_step_us);
            speed_low = substep_calc_speed(state->prev_trans_pos - state->prev_high, state->prev_trans_us - state->prev_step_us);
        } else {
            // otherwise use the slopes from the last transition to now
            speed_high = substep_calc_speed(high - state->prev_trans_pos, step_us - state->prev_trans_us);
            speed_low = substep_calc_speed(low - state->prev_trans_pos, step_us - state->prev_trans_us);
        }
        // make sure the current speed estimate is between the maximum and
        // minimum values obtained from the step slopes
        if (state->speed_2_20 > speed_high)
            state->speed_2_20 = speed_high;
        if (state->speed_2_20 < speed_low)
            state->speed_2_20 = speed_low;

        // convert the speed units from "sub-steps per 2^20 us" to "sub-steps
        // per second"
        state->speed = (state->speed_2_20 * 62500LL) >> 16;

        // estimate the current position by applying the speed estimate to the
        // most recent transition
        state->position = state->prev_trans_pos + (((int64_t)state->speed_2_20 * (step_us - transition_us)) >> 20);

        // make sure the position estimate is between "low" and "high", as we
        // can be sure the actual current position must be in this range
        if ((int)(state->position - high) > 0)
            state->position = high;
        else if ((int)(state->position - low) < 0)
            state->position = low;
    }

    // save the current values to use on the next sample
    state->prev_low = low;
    state->prev_high = high;
    state->raw_step = step;
    state->prev_step_us = step_us;
}




// function to measure the difference between the different steps on the encoder
substep_phases_t substep_calibrate_phases(PIO pio, uint sm)
{
#define sample_count  1024
//#define SHOW_ALL_SAMPLES
#ifdef SHOW_ALL_SAMPLES
    static int result[sample_count];
    int i;
#endif
    int index, cycles, clocks_per_us, calib[4];
    uint cur_us, last_us, step_us, step, last_step;
    int64_t sum[4], total;
    substep_phases_t phases = {
        .first = 0,
        .second = 0,
        .third = 0
    };

    memset(sum, 0, sizeof(sum));

    clocks_per_us = (clock_get_hz(clk_sys) + 500000) / 1000000;

    // keep reading the PIO state in a tight loop to get all steps and use the
    // transition measures of the PIO code to measure the time of each step
    last_step = -10;
    index = -10;
    while (index < sample_count) {

        quadrature_encoder_substep_get_counts(pio, sm, &step, &cycles, &step_us);

        // wait until we have a transition
        if (step == last_step)
            continue;

        // synchronize the index with the lower 2 bits of the current step
        if (index < 0 && index > -4 && (step & 3) == 1)
            index = 0;

        // convert the "time since last transition" to an absolute microsecond
        // timestamp
        if (cycles > 0) {
            printf("error: expected forward motion\n");
            return phases;
        }
        cur_us = step_us + (cycles * 13) / clocks_per_us;

        // if the index is already synchronized, use the step size
        if (index >= 0) {
#ifdef SHOW_ALL_SAMPLES
            result[index] = cur_us - last_us;
#endif
            sum[(step - 1) & 3] += cur_us - last_us;
        }
        index++;

        last_step = step;
        last_us = cur_us;
    }

#ifdef SHOW_ALL_SAMPLES
    printf("full sample table:\n");
    for (i = 0; i < sample_count; i++) {
        printf("%d ", result[i]);
        if ((i & 3) == 3)
            printf("\n");
    }
#endif

    // scale the sizes to a total of 256 to be used as sub-steps
    total = sum[0] + sum[1] + sum[2] + sum[3];
    calib[0] = (sum[0] * 256 + total / 2) / total;
    calib[1] = ((sum[0] + sum[1]) * 256 + total / 2) / total;
    calib[2] = ((sum[0] + sum[1] + sum[2]) * 256 + total / 2) / total;

    // print calibration information
    printf("calibration command:\n\n");
    printf("\tsubstep_set_calibration_data(&state, %d, %d, %d);\n\n", 
        calib[0], calib[1], calib[2]);

    phases.first = calib[0];
    phases.second = calib[1];
    phases.third = calib[2];

    return phases;
}


// set the phase size calibration, use the "substep_calibrate_phases" function
// to get the values. Many encoders (especially low cost ones) have phases that
// don't have the same size. To get good substep accuracy, the code should know
// about this. This is specially important at low speeds with encoders that have
// big phase size differences
void substep_set_calibration_data(substep_state_t *state, int step0, int step1, int step2)
{
    state->calibration_data[0] = 0;
    state->calibration_data[1] = step0;
    state->calibration_data[2] = step1;
    state->calibration_data[3] = step2;
}