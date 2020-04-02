#include <ros/ros.h>
#include <robot_io/DrivetrainRequest.h>
#include <robot_io/DrivetrainStatus.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <linux/i2c-dev.h>
extern "C"{
    #include "drivers/SF0180/SF0180.h"
    #include "drivers/TB6612FNG/TB6612FNG.h"
}

static const std::string SERVICE ="drivetrain_request";
static const std::string STATUS_PUBSHISHER ="drivetrain_status";

/* 90 degrees is the middle, and the rest is offset for the particular servo */
static const uint16_t DRIVETRAIN_STRAIGHT_ANGLE = 90 - 10;

typedef robot_io::DrivetrainRequest::Request  DrivetrainRequest;
typedef robot_io::DrivetrainRequest::Response DrivetrainResponse;

typedef SF0180 Servo;
typedef TB6612FNG MotorController;


static Servo steer_servo;
static MotorController drive_motors;

bool drivetrain_request_handler(DrivetrainRequest &req, DrivetrainResponse &res)
{
    int status;

    status  = TB6612FNG_set_speed(drive_motors, 0, req.power/100.);
    status |= TB6612FNG_set_speed(drive_motors, 1, req.power/100.);
    if(status){
        ROS_WARN("Drivetrain: failed to set speed");
        return false;
    }

    /* angle range is [-45, 45] degrees */
    if(req.angle < -45){
        req.angle = -45;
        ROS_WARN("Drivetrain: Steering angle range is [-45, 45] degrees");
    }else if(req.angle > 45){
        req.angle = 45;
        ROS_WARN("Drivetrain: Steering angle range is [-45, 45] degrees");
    }

    /* positive angle is right, negative is left */
    if(SF0180_set_angle(steer_servo, 180 - DRIVETRAIN_STRAIGHT_ANGLE + req.angle)){
        ROS_WARN("Drivetrain: failed to set steering angle");
        return false;
    }

    return true;
}

void drivetrain_status_publish(ros::Publisher &pub){
    robot_io::DrivetrainStatus status;
    float speed;
    if(TB6612FNG_get_speed(drive_motors, 0, &speed)){
        return;
    }
    status.power = speed*100;
    status.angle = 0;
    pub.publish(status);
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

    /* the pwm controllers should be pointers so that multiple devices can use one device */
    steer_servo.pwm_dev.i2c_dev_fd = drive_motors.pwm_dev.i2c_dev_fd;
    /* which pin is it using */
    steer_servo.pwm = 2;

    /* this initializes the PWM controller, maybe not a good idea */
    if(SF0180_initialize(steer_servo)){
        ROS_ERROR("Drivetrain: Failed to initialize Steering servo");
        return 1;
    }

    /* which pins are they using */
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
    ros::Rate loop_rate(20);


    if(init_drivetrain()){
        ROS_ERROR("Drivetrain: Failed to initialize");
        return 1;
    }


    ros::ServiceServer service = n.advertiseService(SERVICE, drivetrain_request_handler);
    ros::Publisher drivetrain_status_pub = n.advertise<robot_io::DrivetrainStatus>(STATUS_PUBSHISHER, 1);

    ROS_INFO("Drivetrain: Ready");

    while(ros::ok()){
        drivetrain_status_publish(drivetrain_status_pub);

        /* Should spin for duration instead of sleep */
        ros::spinOnce();

        loop_rate.sleep();
    }

    TB6612FNG_close(drive_motors);
    close(drive_motors.pwm_dev.i2c_dev_fd);

    return 0;
}
