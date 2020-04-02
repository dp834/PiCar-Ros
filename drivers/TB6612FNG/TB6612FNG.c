#include "drivers/gpio/gpio.h"
#include "drivers/TB6612FNG/TB6612FNG.h"

int TB6612FNG_initialize(TB6612FNG dev){
    int status;

    status = PCA9685_initialize(dev.pwm_dev);
    if(status){
        return status;
    }

    status = PCA9685_set_PRE_SCALE(dev.pwm_dev, PWM_FREQUENCY, 0);
    if(status){
        return status;
    }

    /* Init gpio pins as output pins */
    if(GPIOExport(dev.motor[0].in1) < 0){
        return -1;
    }
    /* Fails to write without this delay
     * should be okay since it's only during initialization */
    usleep(50*1000);
    if(GPIODirection(dev.motor[0].in1, OUT) < 0){
        return -1;
    }
    /* This robot does not allow for in2 support for either motor.
     * Not sure why they tied the pins high */


    if(GPIOExport(dev.motor[1].in1) < 0){
        return -1;
    }
    usleep(50*1000);
    if(GPIODirection(dev.motor[1].in1, OUT) < 0){
        return -1;
    }

    /* Set some consistent defaults */
    if(TB6612FNG_set_control(dev, 0, CCW) < 0){
        return -1;
    }

    if(TB6612FNG_set_control(dev, 1, CCW) < 0){
        return -1;
    }

    return 0;
}

int TB6612FNG_close(TB6612FNG dev){
    if(GPIOUnexport(dev.motor[0].in1) < 0 || GPIOUnexport(dev.motor[1].in1) < 0){
        return -1;
    }
    return  0;
}

int TB6612FNG_set_control(TB6612FNG dev, uint8_t motor, ControlFunction mode){
    /* Make sure it's a valid motor */
    if(motor != 0 && motor != 1){
        return -1;
    }

    /* Chip pins for IN2 are tied high */
    switch(mode){
        case(SHORT_BRAKE):
            if(GPIOWrite(dev.motor[motor].in1, HIGH) < 0){
                return -1;
            }
            break;
        case(CCW):
            if(GPIOWrite(dev.motor[motor].in1, LOW) < 0){
                return -1;
            }
            break;
        /* Can assign IN2 to LOW */
        case(CW):
        case(STOP):
        default:
        /* Not a valid control mode */
            return -1;
    }

    return 0;
}

ControlFunction TB6612FNG_get_control(TB6612FNG dev, uint8_t motor){
    switch(GPIORead(dev.motor[motor].in1)){
        /* Don't have to read IN2 cause it's tied high */
        case(HIGH):
            return SHORT_BRAKE;
        case(LOW):
            return CCW;
        default:
            return -1;
    }
}

int TB6612FNG_set_speed(TB6612FNG dev, uint8_t motor, float speed){
    /* must select valid motor  */
    if(motor != 0 && motor != 1){
        return -1;
    }
    return PCA9685_set_PWM(dev.pwm_dev, dev.motor[motor].pwm, speed);
}

int TB6612FNG_get_speed(TB6612FNG dev, uint8_t motor, float *speed){
    /* must select valid motor  */
    if(motor != 0 && motor != 1){
        return -1;
    }

    return PCA9685_get_PWM(dev.pwm_dev, dev.motor[motor].pwm, speed);
}

