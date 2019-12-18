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
#define LED0_ON_L   0x06
#define LED0_ON_H   0x07
#define LED0_OFF_L  0x08
#define LED0_OFF_H  0x09
#define LED1_ON_L   0x0a
#define LED1_ON_H   0x0b
#define LED1_OFF_L  0x0c
#define LED1_OFF_H  0x0d
#define LED2_ON_L   0x0e
#define LED2_ON_H   0x0f
#define LED2_OFF_L  0x10
#define LED2_OFF_H  0x11
#define LED3_ON_L   0x12
#define LED3_ON_H   0x13
#define LED3_OFF_L  0x14
#define LED3_OFF_H  0x15
#define LED4_ON_L   0x16
#define LED4_ON_H   0x17
#define LED4_OFF_L  0x18
#define LED4_OFF_H  0x19
#define LED5_ON_L   0x1a
#define LED5_ON_H   0x1b
#define LED5_OFF_L  0x1c
#define LED5_OFF_H  0x1d
#define LED6_ON_L   0x1e
#define LED6_ON_H   0x1f
#define LED6_OFF_L  0x20
#define LED6_OFF_H  0x21
#define LED7_ON_L   0x22
#define LED7_ON_H   0x23
#define LED7_OFF_L  0x24
#define LED7_OFF_H  0x25
#define LED8_ON_L   0x26
#define LED8_ON_H   0x27
#define LED8_OFF_L  0x28
#define LED8_OFF_H  0x29
#define LED9_ON_L   0x2a
#define LED9_ON_H   0x2b
#define LED9_OFF_L  0x2c
#define LED9_OFF_H  0x2d
#define LED10_ON_L  0x2e
#define LED10_ON_H  0x2f
#define LED10_OFF_L 0x30
#define LED10_OFF_H 0x31
#define LED11_ON_L  0x32
#define LED11_ON_H  0x33
#define LED11_OFF_L 0x34
#define LED11_OFF_H 0x35
#define LED12_ON_L  0x36
#define LED12_ON_H  0x37
#define LED12_OFF_L 0x38
#define LED12_OFF_H 0x39
#define LED13_ON_L  0x3a
#define LED13_ON_H  0x3b
#define LED13_OFF_L 0x3c
#define LED13_OFF_H 0x3d
#define LED14_ON_L  0x3e
#define LED14_ON_H  0x3f
#define LED14_OFF_L 0x40
#define LED14_OFF_H 0x41
#define LED15_ON_L  0x42
#define LED15_ON_H  0x43
#define LED15_OFF_L 0x44
#define LED15_OFF_H 0x45

#define ALL_LED_ON_L  0x250
#define ALL_LED_ON_H  0x251
#define ALL_LED_OFF_L 0x252
#define ALL_LED_OFF_H 0x253


/* Blocked when SLEEP bit is logic 0 (in MODE1 reg) */
#define PRE_SCALE     0x254
/* Reserved */
#define TESTMODE      0x255
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

typedef struct{
    uint16_t address;
} PCA9685;

int PCA9685_set_register(PCA9685 dev, uint8_t value);
int PCA9685_set_LED_ON(PCA9685 dev, uint8_t led, uint16_t value);
int PCA9685_set_LED_OFF(PCA9685 dev, uint8_t led, uint16_t value);
int PCA9685_set_LED(PCA9685 dev, uint8_t led, uint32_t value);
