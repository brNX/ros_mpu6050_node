#include <ros/ros.h>
#include "mpu6050.h"

int main(int argc, char **argv){

    ros::init(argc, argv, "mpu_6050");

    ros::NodeHandle pn("~");
    ros::NodeHandle n;

    MPU6050 mpu6050(n,pn);
    ros::spin();
    return 0;
}

