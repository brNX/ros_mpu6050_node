#ifndef MPU6050_H
#define MPU6050_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>

class MPU60X0;
class HMC58X3;

#define MPU_FRAMEID "/imu"

class MPU6050
{
public:
    MPU6050(ros::NodeHandle nh, ros::NodeHandle pnh);
    ~MPU6050();
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    MPU60X0 * accelgyro;
    HMC58X3 * magn;
    bool calibrate;
    bool use_compass;
    ros::Publisher imu_calib_pub;
    ros::Publisher imu_pub;
    ros::Publisher mag_pub;


    // calibration parameters
    int16_t gyro_off_x, gyro_off_y, gyro_off_z;

    void zeroGyro();
    bool calibrate_gyro(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);
    void runOnce(const ros::TimerEvent& event);
};

#endif // MPU6050_H
