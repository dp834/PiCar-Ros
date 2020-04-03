#include <ros/ros.h>
#include <robot_io/CameraMotionRequest.h>
#include <robot_io/CameraMotionStatus.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <linux/i2c-dev.h>
extern "C"{
    #include "drivers/SF0180/SF0180.h"
}

static const std::string SERVICE ="camera_motion_request";
static const std::string STATUS_PUBSHISHER ="camera_motion_status";

/* 90 degrees is the middle, and the rest is offset for the particular servo */
static const uint16_t HORIZONTAL_ZERO_ANGLE = 90 - 10;
static const uint16_t VERTICAL_ZERO_ANGLE   = 90 - 80;


typedef robot_io::CameraMotionRequest::Request  CameraMotionRequest;
typedef robot_io::CameraMotionRequest::Response CameraMotionResponse;

typedef SF0180 Servo;

static Servo horizontal_servo;
static Servo vertical_servo;


bool camera_motion_request_handler(CameraMotionRequest &req, CameraMotionResponse &res)
{
    int status;

    /* angle range is [-100, 80] degrees */
    if(req.horizontal_angle < -100){
        req.horizontal_angle = -100;
        ROS_WARN("Camera Motion: Horizontal angle range is [-80, 100] degrees");
    }else if(req.horizontal_angle > 80){
        req.horizontal_angle = 80;
        ROS_WARN("Camera Motion: Horizontal angle range is [-80, 100] degrees");
    }

    /* angle range is [-10, 70] degrees */
    if(req.vertical_angle < -10){
        req.vertical_angle = -10;
        ROS_WARN("Camera Motion: Vertical angle range is [-10, 70] degrees");
    }else if(req.vertical_angle > 70){
        req.vertical_angle = 70;
        ROS_WARN("Camera Motion: Vertical angle range is [-10, 70] degrees");
    }

    /* positive angle is right, negative is left */
    if(SF0180_set_angle_from_center(horizontal_servo, HORIZONTAL_ZERO_ANGLE, -req.horizontal_angle)){
        ROS_WARN("Camera Motion: failed to set horizontal camera angle");
        return false;
    }

    /* positive angle is up, negative is down */
    if(SF0180_set_angle_from_center(vertical_servo, VERTICAL_ZERO_ANGLE, req.vertical_angle)){
        ROS_WARN("Camera Motion: failed to set vertical camera angle");
        return false;
    }

    return true;
}

void camera_motion_status_publish(ros::Publisher &pub){
    robot_io::CameraMotionStatus status;

    if(SF0180_get_angle_from_center(horizontal_servo, HORIZONTAL_ZERO_ANGLE, &status.horizontal_angle)){
        return;
    }

    if(SF0180_get_angle_from_center(vertical_servo, VERTICAL_ZERO_ANGLE, &status.vertical_angle)){
        return;
    }

    pub.publish(status);
}


int init_camera_motion()
{
    const char device[] = "/dev/i2c-1";

    horizontal_servo.pwm_dev.i2c_dev_fd = open(device, O_RDWR);
    if(horizontal_servo.pwm_dev.i2c_dev_fd < 0){
        ROS_ERROR("Camera Motion: Failed to open I2C device");
        return 1;
    }

    if(ioctl(horizontal_servo.pwm_dev.i2c_dev_fd, I2C_SLAVE, 0x40) < 0) {
        fprintf(stderr, "I2C: Error setting slave address\n");
        return -1;
    }

    /* the pwm controllers should be pointers so that multiple devices can use one device */
    vertical_servo.pwm_dev.i2c_dev_fd = horizontal_servo.pwm_dev.i2c_dev_fd;
    /* which pin is it using */
    vertical_servo.pwm = 0;
    horizontal_servo.pwm = 1;

    /* this initializes the PWM controller, maybe not a good idea
     * Only needs to be called once for all servos using the same
     * PWM controller */
    if(SF0180_initialize(horizontal_servo)){
        ROS_ERROR("Camera Motion: Failed to initialize servos");
        return 1;
    }

    return 0;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_motion_node");
    ros::NodeHandle n;
    ros::Rate loop_rate(20);


    if(init_camera_motion()){
        ROS_ERROR("Camera Motion: Failed to initialize");
        return 1;
    }


    ros::ServiceServer service = n.advertiseService(SERVICE, camera_motion_request_handler);
    ros::Publisher camera_motion_status_pub = n.advertise<robot_io::CameraMotionStatus>(STATUS_PUBSHISHER, 1);

    ROS_INFO("Camera Motion: Ready");

    while(ros::ok()){
        camera_motion_status_publish(camera_motion_status_pub);

        /* Should spin for duration instead of sleep */
        ros::spinOnce();

        loop_rate.sleep();
    }

    close(horizontal_servo.pwm_dev.i2c_dev_fd);
    return 0;
}
