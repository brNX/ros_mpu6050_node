#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>

#define MPU_FRAMEID "/imu"

bool calibrate;
ros::Publisher imu_calib_pub;
ros::ServiceClient * clientptr;

extern "C"{

#include "mpu9150.h"
#include "local_defaults.h"

}

int main(int argc, char **argv){

    ros::init(argc, argv, "mpu_6050");

    ros::NodeHandle pn("~");
    ros::NodeHandle n;


    ROS_INFO("Starting mpu_6050_node...");

    /****
     *IMU parameters
     ***/
    int sample_rate;
    pn.param<int>("frequency", sample_rate ,DEFAULT_SAMPLE_RATE_HZ);
    int i2c_bus;
    pn.param<int>("i2c_bus",i2c_bus,0);
    int yaw_mix_factor;
    pn.param<int>("yaw_mix_factor",yaw_mix_factor,DEFAULT_YAW_MIX_FACTOR);

    ROS_INFO("setting up MPU60X0...");

    mpudata_t mpu;

    //mpu9150_set_debug(1);
    ROS_INFO("Initialize MPU_6050...");
    if (mpu9150_init(i2c_bus,sample_rate, yaw_mix_factor)){
        ROS_FATAL("MPU6050 - %s - MPU6050 connection failed",__FUNCTION__);
        ROS_BREAK();
    }
    memset(&mpu, 0, sizeof(mpudata_t));
    if (sample_rate == 0)
        ROS_BREAK();


    ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu/data", 10);
    ros::Rate r(sample_rate);

    while(ros::ok())
    {
        ros::Time now = ros::Time::now();

        sensor_msgs::Imu imu_msg;
        imu_msg.header.stamp = now;
        imu_msg.header.frame_id = MPU_FRAMEID;

         if (mpu9150_read(&mpu) == 0) {

             imu_msg.orientation.x=mpu.fusedQuat[QUAT_X];
             imu_msg.orientation.y=mpu.fusedQuat[QUAT_Y];
             imu_msg.orientation.z=mpu.fusedQuat[QUAT_Z];
             imu_msg.orientation.w=mpu.fusedQuat[QUAT_W];


             //TODO: verify conversion

             float ax_f, ay_f, az_f;
             float gx_f, gy_f, gz_f;

             ax_f =((float) mpu.calibratedAccel[0]) / (16384 / 9.807); // 2g scale in m/s^2
             ay_f =((float) mpu.calibratedAccel[1]) / (16384 / 9.807); // 2g scale in m/s^2
             az_f =((float) mpu.calibratedAccel[2]) / (16384 / 9.807); // 2g scale in m/s^2

             gx_f=((float) mpu.rawGyro[0]) / 16.4f; // for degrees/s 2000 scale
             gy_f=((float) mpu.rawGyro[1]) / 16.4f; // for degrees/s 2000 scale
             gz_f=((float) mpu.rawGyro[2]) / 16.4f; // for degrees/s 2000 scale

             imu_msg.linear_acceleration.x=ax_f;
             imu_msg.linear_acceleration.y=ay_f;
             imu_msg.linear_acceleration.z=az_f;

             imu_msg.angular_velocity.x=gx_f;
             imu_msg.angular_velocity.y=gy_f;
             imu_msg.angular_velocity.z=gz_f;

             imu_pub.publish(imu_msg);


         }else{
             ROS_WARN("MPU6050 - %s - MPU6050 read failed",__FUNCTION__);
         }


        /*accelgyro->getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        // remove offsets from the gyroscope
        gx -=  gyro_off_x;
        gy -=  gyro_off_y;
        gz -=  gyro_off_z;

        float ax_f, ay_f, az_f;
        float gx_f, gy_f, gz_f;

        //TODO: verify this
        gx_f=((float) gx) / (131 * 57.3); // for radian/s 250 scale
        gy_f=((float) gy) / (131 * 57.3); // for radian/s 250 scale
        gz_f=((float) gz) / (131 * 57.3); // for radian/s 250 scale

        ax_f =((float) ax) / (16384 / 9.807); // 2g scale in m/s^2
        ay_f =((float) ay) / (16384 / 9.807); // 2g scale in m/s^2
        az_f =((float) az) / (16384 / 9.807); // 2g scale in m/s^2


        //values[i] = ((float) accgyroval[i]) / 16.4f; // NOTE: this depends on the sensitivity chosen


        imu_msg.linear_acceleration.x=ax_f;
        imu_msg.linear_acceleration.y=ay_f;
        imu_msg.linear_acceleration.z=az_f;

        imu_msg.angular_velocity.x=gx_f;
        imu_msg.angular_velocity.y=gy_f;
        imu_msg.angular_velocity.z=gz_f;

        imu_pub.publish(imu_msg);*/


        ros::spinOnce();
        r.sleep();
    }



    mpu9150_exit();

    return 0;
}

