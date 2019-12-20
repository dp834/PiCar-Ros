#include "drivers/PCA9685/PCA9685.h"

typedef struct{
    PCA9685 pwm_dev;
    uint8_t pwm;
} SF0180;

/* angle in degrees */
int SF0180_set_angle(SF0180 dev, uint16_t angle);

