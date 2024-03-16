/**
 * @file motor.h
 * @author Alper Tunga Güven (alpertunga2003@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-11-11
 *
 * @copyright Copyright (c) 2023
 *
 * Motor Shield Link:
 * https://www.amazon.com.tr/Akozon-DC5-12V-0A-30A-Dual-channel-Arduino/dp/B07H2MDXMN/ref=asc_df_B07H2MDXMN/?tag=trshpngglede-21&linkCode=df0&hvadid=510499475756&hvpos=&hvnetw=g&hvrand=11669362855173271918&hvpone=&hvptwo=&hvqmt=&hvdev=c&hvdvcmdl=&hvlocint=&hvlocphy=1012783&hvtargid=pla-703073782452&psc=1
 *
 */

#ifndef MOTOR_HPP
#define MOTOR_HPP

class PwmController {
public:
    PwmController() = delete;

    PwmController(const uint pwmPin, const uint frequencyHz);

    void setDutyCycle(const float dutyCycle) const;
    
    void setDutyCycle2(const float dutyCycle) const;

    void setNs(const uint32_t nanos) const;

private:
    const uint mPwmPin {};
    uint mFrequencyHz {};
    uint16_t mWrap {};
};

class Motor {

public:

    Motor() = delete;

    Motor(const uint pwmPinL, const uint pwmPinR);

    void setSpeedPercent(const float speedPercent) const;

private:
    const PwmController mPwmL;
    const PwmController mPwmR;

};

#endif