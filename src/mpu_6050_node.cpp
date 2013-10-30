#include <ros/ros.h>
#include "sensor_msgs/Imu.h"
#include "MPU60X0/MPU60X0.h"
#include "i2c_ros/i2c.h"

#define MPU_FRAMEID "/base_mpu_6050"

int main(int argc, char **argv){
    ros::init(argc, argv, "mpu_6050");

    ros::NodeHandle pn("~");
    ros::ServiceClient client = pn.serviceClient<i2c_ros::i2c>("i2c_operation");

    /****
     *IMU parameters
     ***/
    int frequency;
    pn.param<int>("frequency", frequency ,20);
    //pn.param<bool>("use_compass", use_compass,true);

    MPU60X0 accelgyro(client);


    // verify connection
    ROS_DEBUG("Testing device connections...");
    ROS_DEBUG(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");


    ros::Publisher imu_pub = pn.advertise<sensor_msgs::Imu>("imu/data", 10);
    ros::Rate r(frequency);

    while(ros::ok())
    {
        ros::Time now = ros::Time::now();

        sensor_msgs::Imu imu_msg;
        imu_msg.header.stamp = now;
        imu_msg.header.frame_id = MPU_FRAMEID;



        /*sensor_msgs::Imu imu_msg = mti->fillImuMessage(now);
        mti_pub.publish(imu_msg);

        sensor_msgs::NavSatFix nav_fix_msg = mti->fillNavFixMessage(now);
        navsat_pub.publish(nav_fix_msg);

        nav_msgs::Odometry odom_msg = mti->fillOdometryMessage(listener, odom_broadcaster, now);
        odomPub.publish(odom_msg);*/

        ros::spinOnce();
        r.sleep();
    }





    return 0;
}

