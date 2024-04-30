/**
 * @file motor.cpp
 * @author Alper Tunga Güven (alpertunga2003@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-11-11
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include <cstdio>

#include "motor.hpp"

PwmController::PwmController(const uint pwmPin, const uint frequencyHz) : mPwmPin(pwmPin), mFrequencyHz(frequencyHz) {
    gpio_set_function(mPwmPin, GPIO_FUNC_PWM);

    pwm_config config { pwm_get_default_config() };

    uint32_t clockSpeed = clock_get_hz(clk_sys);

    const uint32_t periodNs { static_cast<uint32_t>(1000000000.0 / frequencyHz) };
    printf("Pwm period in ns: %u\n", periodNs);
    float clockDiv { 1.0f };
    if (periodNs >= 1000000) {
        clockDiv = static_cast<float>(clockSpeed / 1000000U); // For 1us clock cycle.
        mWrap = (clockSpeed / clockDiv * static_cast<float>(periodNs)) / 1000000000.0;
    } else {
        mWrap = (clockSpeed / clockDiv * static_cast<float>(periodNs)) / 1000000000.0;
    }
    printf("Pwm wrap value: %u\n", mWrap);

    pwm_config_set_clkdiv(&config, clockDiv);
    pwm_config_set_wrap(&config, mWrap);
    const uint sliceNum { pwm_gpio_to_slice_num(mPwmPin) };

    pwm_init(sliceNum, &config, true);
}

void PwmController::setDutyCycle(const float dutyCycle) const {
    const uint32_t periodNs { static_cast<uint32_t>(1000000000.0 / mFrequencyHz) };
    const float multiplier { static_cast<float>(periodNs) / 100.0f };
    printf("Duty Cycle = %.2f , Multiplier = %.2f\n", dutyCycle, multiplier);
    PwmController::setNs(static_cast<uint32_t>(dutyCycle * multiplier));
}

void PwmController::setDutyCycle2(const float dutyCycle) const {
    const uint chanLevel { static_cast<uint>(mWrap * (dutyCycle / 100.0f)) };
    pwm_set_chan_level(pwm_gpio_to_slice_num(mPwmPin), mPwmPin % 2, chanLevel);
    #ifndef NDEBUG
    printf("Debug Messeage => Pwm %u Set with chanLevel: %u and dutyCycle:%3.2f\n", mPwmPin, chanLevel, dutyCycle);
    #endif
}

void PwmController::setNs(const uint32_t nanos) const {
    const uint32_t periodNs { static_cast<uint32_t>(1000000000.0 / mFrequencyHz) };
    /* Calculate the pwm "on" period. */
    uint16_t chanLevel { static_cast<uint16_t>((nanos * mWrap) / periodNs) };
    /* Check if the needed channel level is within the accepted range. */
    pwm_set_chan_level(pwm_gpio_to_slice_num(mPwmPin), mPwmPin % 2, chanLevel);
    #ifndef NDEBUG
    printf("Debug Messeage => Pwm %u Set with chanLevel: %u and nanos: %u\n", mPwmPin, chanLevel, nanos);
    #endif
}

Motor::Motor(const uint pwmPinL, const uint pwmPinR) :
    mPwmL(pwmPinL, 10000), mPwmR(pwmPinR, 10000) {

    mPwmL.setDutyCycle(0);
    mPwmR.setDutyCycle(0);

}

void Motor::setSpeedPercent(const float speedPercent) const {
    if (speedPercent >= 0) {
        mPwmL.setDutyCycle2(speedPercent);
        mPwmR.setDutyCycle2(0);
    } else {
        mPwmL.setDutyCycle2(0);
        mPwmR.setDutyCycle2(-speedPercent);
    }
}
