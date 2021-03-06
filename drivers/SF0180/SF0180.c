#include "SF0180.h"
#include <stdio.h>
#include <inttypes.h>

/* angle in degrees */
int SF0180_set_angle(SF0180 dev, uint16_t angle)
{
    /* Neutral point = 1500 micro seconds
     * Operating angle = 90° +- 5°
     * Pulse Width Range = 750~2250 micro seconds
     * Maximum Travel = approx 140° (in Pulse width range)
     */

    float pulse_width;

    /* if angle requested is out of range dont do anything */
    if(angle < MIN_ANGLE || angle > MAX_ANGLE){
        return -1;
    }

    /* angle/(max_angle - min_angle) * (max_width - min_width) + min_width
     * convert angle to a percentage of range and scale to the new range */
    pulse_width  = (angle - MIN_ANGLE)/(float)(MAX_ANGLE - MIN_ANGLE);
    pulse_width *= (PULSE_WIDTH_MAX - PULSE_WIDTH_MIN);
    pulse_width += PULSE_WIDTH_MIN;

    /* Pulse width currently in micro seconds */

    /* Get the on ratio of being on
     * frequency * pulse_width gives percent on
     * Percentage = Time on / Period
     * Since frequency = 1/period */

    pulse_width *= PWM_FREQUENCY;
    /* Get pulse width as a fraction of a second (duty cycle) */
    pulse_width /= 1000000;


    return PCA9685_set_PWM(dev.pwm_dev, dev.pwm, pulse_width);
}

int SF0180_get_angle(SF0180 dev, uint16_t *angle)
{
    float pulse_width;
    int status;

    status = PCA9685_get_PWM(dev.pwm_dev, dev.pwm, &pulse_width);
    if(status){
        return status;
    }

    /* undo conversion from the set_angle function above */
    pulse_width *= 1000000;
    pulse_width /= PWM_FREQUENCY;
    pulse_width -= PULSE_WIDTH_MIN;
    pulse_width /= (PULSE_WIDTH_MAX - PULSE_WIDTH_MIN);

    /* Not sure why the angle is one below what it should be */
    *angle = pulse_width*(MAX_ANGLE - MIN_ANGLE) + MIN_ANGLE + 1;

    return 0;
}

int SF0180_set_angle_from_center(SF0180 dev, uint16_t center, int16_t angle)
{
    angle += center;
    if(angle < 0){
        return -1;
    }

    return SF0180_set_angle(dev, angle);
}
int SF0180_get_angle_from_center(SF0180 dev, uint16_t center, int16_t *rel_angle)
{
    int status;
    uint16_t angle;
    status = SF0180_get_angle(dev, &angle);
    if(status){
        return status;
    }

    *rel_angle = angle - center;

    return 0;
}

int SF0180_initialize(SF0180 dev)
{
    int status;

    status = PCA9685_initialize(dev.pwm_dev);
    if(status){
        return status;
    }

    status = PCA9685_set_PRE_SCALE(dev.pwm_dev, PWM_FREQUENCY, 0);
    if(status){
        return status;
    }

    return 0;
}
