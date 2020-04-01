#include <ros/ros.h>
#include <robot_io/DriveTrainRequest.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <linux/i2c-dev.h>
extern "C"{
    #include "drivers/SF0180/SF0180.h"
    #include "drivers/TB6612FNG/TB6612FNG.h"
}

static const std::string SERVICE ="drive_train_request";

typedef robot_io::DriveTrainRequest::Request  DriveTrainRequest;
typedef robot_io::DriveTrainRequest::Response DriveTrainResponse;

typedef SF0180 Servo;
typedef TB6612FNG MotorController;


static Servo steer_servo;
static MotorController drive_motors;

bool drivetrain_request_handler(DriveTrainRequest &req, DriveTrainResponse &res)
{
    int status;

    status = TB6612FNG_set_speed(drive_motors, 0, req.power/100.);
    status |= TB6612FNG_set_speed(drive_motors, 1, req.power/100.);
    if(status){
        ROS_WARN("Drivetrain: failed to set speed");
        return false;
    }

    return true;
}


int init_drivetrain()
{
    const char device[] = "/dev/i2c-1";

    drive_motors.pwm_dev.i2c_dev_fd = open(device, O_RDWR);
    if(drive_motors.pwm_dev.i2c_dev_fd < 0){
        ROS_ERROR("Drivetrain: Failed to open I2C device");
        return 1;
    }

    if(ioctl(drive_motors.pwm_dev.i2c_dev_fd, I2C_SLAVE, 0x40) < 0) {
        fprintf(stderr, "I2C: Error setting slave address\n");
        return -1;
    }

    int status = PCA9685_initialize(drive_motors.pwm_dev);
    if(status){
        ROS_ERROR("Drivetrain: Failed to initialize PWM driver:%d", status);
        return 1;
    }

    drive_motors.motor[0].in1 = 17;
    drive_motors.motor[0].pwm = 5;
    drive_motors.motor[1].in1 = 27;
    drive_motors.motor[1].pwm = 4;


    if(TB6612FNG_initialize(drive_motors)){
        ROS_ERROR("Drivetrain: Failed to initialize Motor controller");
        return 1;
    }

    return 0;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "drivetrain_node");
    ros::NodeHandle n;


    if(init_drivetrain()){
        ROS_ERROR("Drivetrain: Failed to initialize");
        return 1;
    }


    ros::ServiceServer service = n.advertiseService(SERVICE, drivetrain_request_handler);
    ROS_INFO("Drivetrain: Ready");

    ros::spin();

    TB6612FNG_close(drive_motors);
    close(drive_motors.pwm_dev.i2c_dev_fd);

    return 0;
}
