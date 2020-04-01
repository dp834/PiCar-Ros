#ifndef SF0180_H
#define SF0180_H

#include "drivers/PCA9685/PCA9685.h"

/* In degrees */

#define MIN_ANGLE 0
#define MAX_ANGLE 180

/* In micro seconds */
#define PULSE_WIDTH_MIN 1000
#define PULSE_WIDTH_MAX 2000

/* In Hz */
#define PWM_FREQUENCY 50

typedef struct{
    PCA9685 pwm_dev;
    uint8_t pwm;
} SF0180;

int SF0180_initialize(SF0180 dev);
/* angle in degrees */
int SF0180_set_angle(SF0180 dev, uint16_t angle);

#endif
