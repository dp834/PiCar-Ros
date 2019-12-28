#include <stdint.h>


/* All registers */

#define MODE1       0x00
#define MODE2       0x01
#define SUBADR1     0x02
#define SUBADR2     0x03
#define SUBADR3     0x04
#define ALLCALLADR  0x05

/*
 * XXX_ON_L  control byte 0
 * XXX_ON_H  control byte 1
 * XXX_OFF_L control byte 2
 * XXX_OFF_H control byte 3
 */
#define PWM0_ON_L   0x06
#define PWM0_ON_H   0x07
#define PWM0_OFF_L  0x08
#define PWM0_OFF_H  0x09
#define PWM1_ON_L   0x0a
#define PWM1_ON_H   0x0b
#define PWM1_OFF_L  0x0c
#define PWM1_OFF_H  0x0d
#define PWM2_ON_L   0x0e
#define PWM2_ON_H   0x0f
#define PWM2_OFF_L  0x10
#define PWM2_OFF_H  0x11
#define PWM3_ON_L   0x12
#define PWM3_ON_H   0x13
#define PWM3_OFF_L  0x14
#define PWM3_OFF_H  0x15
#define PWM4_ON_L   0x16
#define PWM4_ON_H   0x17
#define PWM4_OFF_L  0x18
#define PWM4_OFF_H  0x19
#define PWM5_ON_L   0x1a
#define PWM5_ON_H   0x1b
#define PWM5_OFF_L  0x1c
#define PWM5_OFF_H  0x1d
#define PWM6_ON_L   0x1e
#define PWM6_ON_H   0x1f
#define PWM6_OFF_L  0x20
#define PWM6_OFF_H  0x21
#define PWM7_ON_L   0x22
#define PWM7_ON_H   0x23
#define PWM7_OFF_L  0x24
#define PWM7_OFF_H  0x25
#define PWM8_ON_L   0x26
#define PWM8_ON_H   0x27
#define PWM8_OFF_L  0x28
#define PWM8_OFF_H  0x29
#define PWM9_ON_L   0x2a
#define PWM9_ON_H   0x2b
#define PWM9_OFF_L  0x2c
#define PWM9_OFF_H  0x2d
#define PWM10_ON_L  0x2e
#define PWM10_ON_H  0x2f
#define PWM10_OFF_L 0x30
#define PWM10_OFF_H 0x31
#define PWM11_ON_L  0x32
#define PWM11_ON_H  0x33
#define PWM11_OFF_L 0x34
#define PWM11_OFF_H 0x35
#define PWM12_ON_L  0x36
#define PWM12_ON_H  0x37
#define PWM12_OFF_L 0x38
#define PWM12_OFF_H 0x39
#define PWM13_ON_L  0x3a
#define PWM13_ON_H  0x3b
#define PWM13_OFF_L 0x3c
#define PWM13_OFF_H 0x3d
#define PWM14_ON_L  0x3e
#define PWM14_ON_H  0x3f
#define PWM14_OFF_L 0x40
#define PWM14_OFF_H 0x41
#define PWM15_ON_L  0x42
#define PWM15_ON_H  0x43
#define PWM15_OFF_L 0x44
#define PWM15_OFF_H 0x45

#define ALL_PWM_ON_L  0xFA
#define ALL_PWM_ON_H  0xFB
#define ALL_PWM_OFF_L 0xFC
#define ALL_PWM_OFF_H 0xFD


/* Blocked when SLEEP bit is logic 0 (in MODE1 reg) */
#define PRE_SCALE     0xFE
/* Reserved */
#define TESTMODE      0xFF
/**
 * Everything else is also reserved
 * Auto Increment past register 69 will point to MODE1 register (register 0)
 * Auto Increment also works from register 250 to register 254, then rolls over to register 0
 */

/*******************************************************/

/*
 * MODE1 register options
 * to set options & them together
 */

#define MODE1_RESTART   (1<<7)
#define MODE1_EXTCLK    (1<<6)
#define MODE1_AI        (1<<5)
#define MODE1_SLEEP     (1<<4)
#define MODE1_SUB1      (1<<3)
#define MODE1_SUB2      (1<<2)
#define MODE1_SUB3      (1<<1)
#define MODE1_ALLCALL   (1<<0)

/*
 * MODE 2 register options
 * to set options & them together
 */

#define MODE2_7         (1<<7)
#define MODE2_6         (1<<6)
#define MODE2_5         (1<<5)
#define MODE2_INVRT     (1<<4)
#define MODE2_OCH       (1<<3)
#define MODE2_OUTDRV    (1<<2)
#define MODE2_OUTNE_1   (1<<1)
#define MODE2_OUTNE_0   (1<<0)

/* Get pwm base offset by its number */

#define PWM_BASE(n)  (6+n*4)
#define ON_L_OFFSET  0
#define ON_H_OFFSET  1
#define OFF_L_OFFSET 2
#define OFF_H_OFFSET 3

typedef struct{
    int i2c_dev_fd;
} PCA9685;

int PCA9685_initialize(PCA9685 dev);
int PCA9685_set_register(PCA9685 dev, uint8_t reg, uint8_t value);
int PCA9685_set_PWM_ON(PCA9685 dev, uint8_t pwm, uint16_t value);
int PCA9685_set_PWM_OFF(PCA9685 dev, uint8_t pwm, uint16_t value);
int PCA9685_set_PRE_SCALE(PCA9685 dev, uint32_t frequency, uint32_t clock);
int PCA9685_set_PWM(PCA9685 dev, uint8_t pwm, float percentage);
int PCA9685_set_PWM_with_shift(PCA9685 dev, uint8_t pwm, float percentage, uint8_t shift_percentage);
int PCA9685_set_on(PCA9685 dev, uint8_t pwm);
int PCA9685_set_off(PCA9685 dev, uint8_t pwm);
