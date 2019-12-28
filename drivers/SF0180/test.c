#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include "linux/i2c-dev.h"

#include "drivers/SF0180/SF0180.h"

int main(int argc, char **argv){
    char device[] = "/dev/i2c-1";
    int fd;
    PCA9685 pwm_controller;
    SF0180  servo;
    unsigned int angle;
    unsigned int pwm_id;
    int status;

    if(argc != 3){
        fprintf(stderr, "Usage: %s number angle\n", argv[0]);
        return -1;
    }


    pwm_id = atoi(argv[1]);
    angle = atoi(argv[2]);

    if(pwm_id > 15){
        fprintf(stderr, "servo number out must be in range [0, 15]");
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

    servo.pwm_dev = pwm_controller;
    servo.pwm = pwm_id;

    status = SF0180_initialize(servo);
    if(status){
        fprintf(stderr, "Servo: failed to initialize\n");
        return status;
    }

    status = SF0180_set_angle(servo, angle);
    if(status){
        fprintf(stderr, "Servo: failed to set angle\n");
        return status;
    }

    close(servo.pwm_dev.i2c_dev_fd);
    return 0;
}
