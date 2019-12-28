#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include "linux/i2c-dev.h"

#include "PCA9685.h"




int main(int argc, char **argv){
    char device[] = "/dev/i2c-1";
    int fd;
    PCA9685 pwm_controller;
    char command;
    int pwm_id;

    if(argc != 3){
        fprintf(stderr, "Usage: %s command number\n", argv[0]);
        return -1;
    }


    command = argv[1][0];
    pwm_id = atoi(argv[2]);

    if(pwm_id >15){
        fprintf(stderr, "pwm_id out must be in range [0, 15]");
        return -1;
    }

    fd = open(device, O_RDWR);
    if(fd < 0) {
        fprintf(stderr, "I2C: Error opening device\n");
        return -1;
    }

    if(ioctl(fd, I2C_SLAVE, 0x40) < 0) {
        fprintf(stderr, "I2C: Error setting slave address\n");
		return -1;
    }
    
    pwm_controller.i2c_dev_fd = fd;

    if(PCA9685_initialize(pwm_controller)){
        return -1;
    }

    switch(command){
        /* enable */
        case('e'):
            return PCA9685_set_on(pwm_controller, pwm_id);
            break;
        /* disable */
        case('d'):
            return PCA9685_set_off(pwm_controller, pwm_id);
            break;
        case('p'):
            return PCA9685_set_PWM(pwm_controller, pwm_id, 30);
        default:
            break;
    }


    close(pwm_controller.i2c_dev_fd);

    return 0;
}
