#ifndef TB6612FNG_H
#define TB6612FNG_H

#include <stdint.h>
#include "drivers/PCA9685/PCA9685.h"

/* Should match servo frequency (SF0180) */
#define PWM_FREQUENCY  50

typedef enum {SHORT_BRAKE, CCW, CW, STOP}ControlFunction;


typedef struct{
    PCA9685 pwm_dev;
    struct {
        /* gpio pin number corresponding to IN1 */
        uint8_t in1;
        /* gpio pin number corresponding to IN2 */
        uint8_t in2;
        /* pwm signals output pin from pwm controller*/
        uint8_t pwm;
    } motor[2];
} TB6612FNG;

int TB6612FNG_initialize(TB6612FNG dev);
int TB6612FNG_close(TB6612FNG dev);
int TB6612FNG_set_control(TB6612FNG dev, uint8_t motor, ControlFunction mode);
ControlFunction TB6612FNG_get_control(TB6612FNG dev, uint8_t motor);
int TB6612FNG_set_speed(TB6612FNG dev, uint8_t motor, float percentage);

#endif
