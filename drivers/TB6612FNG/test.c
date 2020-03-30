#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include "linux/i2c-dev.h"

#include "drivers/TB6612FNG/TB6612FNG.h"

int main(int argc, char **argv){
    char device[] = "/dev/i2c-1";
    int fd;
    TB6612FNG motor_controller;
    int speed;
    ControlFunction mode;
    int status;

    if(argc != 3){
        fprintf(stderr, "Usage: %s mode speed\nmodes: 0:SHORT_BRAKE\n       1:CCW\nspeed: percentage on\n", argv[0]);
        return -1;
    }

    switch(atoi(argv[1])){
        case(0):
            mode = SHORT_BRAKE;
            break;
        case(1):
            mode = CCW;
            break;
        default:
            fprintf(stderr, "Invalid mode, must be 0 or 1\n");
            return -1;
    }

    speed = atoi(argv[2]);

    fd = open(device, O_RDWR);
    if(fd < 0) {
        fprintf(stderr, "I2C: Error opening device\n");
        return -1;
    }

    if(ioctl(fd, I2C_SLAVE, 0x40) < 0) {
        fprintf(stderr, "I2C: Error setting slave address\n");
        return -1;
    }

    motor_controller.pwm_dev.i2c_dev_fd = fd;


    if(PCA9685_initialize(motor_controller.pwm_dev)){
        return -1;
    }

    motor_controller.motor[0].in1 = 17;
    motor_controller.motor[0].pwm = 5;
    motor_controller.motor[1].in1 = 27;
    motor_controller.motor[1].pwm = 4;

    status = TB6612FNG_initialize(motor_controller);
    if(status){
        fprintf(stderr, "Motor controller: failed to initialize\n");
        return status;
    }

    status  = TB6612FNG_set_control(motor_controller, 0, mode);
    status |= TB6612FNG_set_control(motor_controller, 1, mode);
    if(status){
        fprintf(stderr, "Motor Controller: failed to set mode\n");
        return status;
    }

    status  = TB6612FNG_set_speed(motor_controller, 0, speed/100.);
    status |= TB6612FNG_set_speed(motor_controller, 1, speed/100.);
    if(status){
        fprintf(stderr, "Motor Controller: failed to set speed\n");
        return status;
    }


    TB6612FNG_close(motor_controller);
    close(motor_controller.pwm_dev.i2c_dev_fd);
    return 0;
}
