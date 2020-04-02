#include "PCA9685.h"
#include <stdio.h>
#include "linux/i2c-dev.h"
#include <byteswap.h>
#include <unistd.h>

int PCA9685_initialize(PCA9685 dev)
{
    /* Autoincrement and normal mode enabled */
    return PCA9685_set_register(dev, MODE1, 0x20);
}

int PCA9685_set_register(PCA9685 dev, uint8_t reg, uint8_t value)
{
    return i2c_smbus_write_byte_data(dev.i2c_dev_fd, reg, value);
}

int PCA9685_get_register(PCA9685 dev, uint8_t reg, uint8_t *value)
{
    int32_t status;
    status = i2c_smbus_read_byte_data(dev.i2c_dev_fd, reg);
    if(status == -1){
        return status;
    }
    *value = status;
    return 0;
}

int PCA9685_set_on(PCA9685 dev, uint8_t pwm)
{
    int status;
    status = PCA9685_set_PWM_OFF(dev, pwm, 0x0000);
    if(status){
        return status;
    }
    return PCA9685_set_PWM_ON(dev, pwm, 0x1000);
}

int PCA9685_set_off(PCA9685 dev, uint8_t pwm)
{
    int status;
    status = PCA9685_set_PWM_ON(dev, pwm, 0x0000);
    if(status){
        return status;
    }
    return PCA9685_set_PWM_OFF(dev, pwm, 0x1000);
}

int PCA9685_set_PWM_ON(PCA9685 dev, uint8_t pwm, uint16_t value)
{
    /* top 3 bits are reserved */
    value &= 0x1FFF;

    return i2c_smbus_write_word_data(dev.i2c_dev_fd, PWM_BASE(pwm) + ON_L_OFFSET, value);
}

int PCA9685_set_PWM_OFF(PCA9685 dev, uint8_t pwm, uint16_t value)
{
    /* top 3 bits are reserved */
    value &= 0x1FFF;
    return i2c_smbus_write_word_data(dev.i2c_dev_fd, PWM_BASE(pwm) + OFF_L_OFFSET, value);
}

/*
 * frequency: target frequency in Hz
 * clock: speed of the clock used(MHz), default is 25. Set to zero for default
 */
int PCA9685_set_PRE_SCALE(PCA9685 dev, uint32_t frequency, uint32_t clock)
{
    uint32_t calc;
    uint8_t prescaler;
    uint8_t reg_val;
    int status;

    if(clock == 0){
        /* default clock speed is 25MHz */
        clock = 25;
    }

    calc = clock * (1000000/4096) /frequency - 1;
    if(calc < 0x03 || calc > 0xFF){
        /* Cannot find prescaler value in range */
        return -1;
    }

    prescaler = calc & 0xFF;

    /* Prescale can only be set when sleep bit of MODE1 is set to logic 1 */
    status = PCA9685_get_register(dev, MODE1, &reg_val);
    if(status){
        return status;
    }

    status = PCA9685_set_register(dev, MODE1, reg_val | MODE1_SLEEP);
    if(status){
        return status;
    }

    status = i2c_smbus_write_byte_data(dev.i2c_dev_fd, PRE_SCALE, prescaler);
    if(status){
        return status;
    }

    /* now in restart mode, see section 7.3.1.1 of PCA9685 manual */
    /* read MODE1 register */
    status = PCA9685_get_register(dev, MODE1, &reg_val);
    if(status){
        return status;
    }

    if(!(reg_val & MODE1_RESTART)){
        /* we are done */
        return 0;
    }

    /* clear sleep bit */
    reg_val &= ~MODE1_SLEEP;
    status = PCA9685_set_register(dev, MODE1, reg_val);
    if(status){
        return status;
    }

    /* let oscillator stabilize) */
    usleep(500);

    /* write logic 1 to restart bit to clear */
    reg_val |= MODE1_RESTART;
    status = PCA9685_set_register(dev, MODE1, reg_val);
    if(status){
        return status;
    }

    return 0;
}

int PCA9685_set_PWM(PCA9685 dev, uint8_t pwm, float percentage)
{
    return PCA9685_set_PWM_with_shift(dev, pwm, percentage, 0);
}

/* compared to internal clock counting  [0, 4095] */
int PCA9685_set_PWM_with_shift(PCA9685 dev, uint8_t pwm, float percentage,
                               uint8_t shift_percentage)
{
    uint16_t on_start;
    uint16_t off_start;
    uint16_t on_amount;
    uint8_t data[4];

    if(percentage >= 1){
        /* assume they want full power
         * On time can't be the full duration */
        percentage = .999999;
    }

    on_amount = 4096 * percentage;
    on_start  = 4096 * shift_percentage;
    off_start = on_start + on_amount;

    data[0] = on_start & 0xFF;
    data[1] = (on_start >> 8) & 0xFF;
    data[2] = off_start & 0xFF;
    data[3] = (off_start >> 8) & 0xFF;

    return i2c_smbus_write_i2c_block_data(dev.i2c_dev_fd, PWM_BASE(pwm), 4, data);
}

int PCA9685_get_PWM(PCA9685 dev, uint8_t pwm, float *percentage)
{
    int32_t status;

    uint16_t on_start  = 0;
    uint16_t off_start = 0;
    uint16_t on_amount = 0;
    uint8_t data[4];

    status = i2c_smbus_read_i2c_block_data(dev.i2c_dev_fd, PWM_BASE(pwm), 4, data);

    if(status == -1){
        return status;
    }

    on_start   = data[0];
    on_start  |= data[1] << 8;

    off_start  = data[2];
    off_start |= data[3] << 8;

    on_amount = off_start - on_start;

    if(on_amount == 4095){
        *percentage = 1;
    }else{
        *percentage = on_amount / 4096.;
    }

    return 0;
}

