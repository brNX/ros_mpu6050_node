#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include "MPU60X0/MPU60X0.h"
#include "HMC58X3/HMC58X3.h"
#include "i2ckernel.h"

#define MPU_FRAMEID "/mpu_6050"

MPU60X0 * accelgyro;
bool calibrate;
ros::Publisher imu_calib_pub;

// calibration parameters
int16_t gyro_off_x=0, gyro_off_y=0, gyro_off_z=0;
//float acc_scale_x, acc_scale_y, acc_scale_z, magn_scale_x, magn_scale_y, magn_scale_z;

/**
 * Computes gyro offsets
*/
void zeroGyro() {
    ROS_INFO("calibrating gyro");
    const int totSamples = 3;
    int16_t ax, ay, az;
    int16_t gx, gy, gz;

    float tmpOffsets[] = {0,0,0};

    for (int i = 0; i < totSamples; i++){
        accelgyro->getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        tmpOffsets[0] += gx;
        tmpOffsets[1] += gy;
        tmpOffsets[2] += gz;
    }

    gyro_off_x = tmpOffsets[0] / totSamples;
    gyro_off_y = tmpOffsets[1] / totSamples;
    gyro_off_z = tmpOffsets[2] / totSamples;
}

bool calibrate_gyro(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp){
    zeroGyro();

    //publish calibration status
    calibrate=true;
    std_msgs::Bool calibrated;
    calibrated.data=calibrate;
    imu_calib_pub.publish(calibrated);

    return true;
}


int main(int argc, char **argv){

    ros::init(argc, argv, "mpu_6050");

    ros::NodeHandle pn("~");
    ros::NodeHandle n;


    ROS_INFO("Starting mpu_6050...");


    /****
     *IMU parameters
     ***/
    int frequency;
    std::string device;
    bool use_compass;
    pn.param<int>("frequency", frequency ,20);
    pn.param<bool>("autocalibrate",calibrate,true);
    pn.param<std::string>("device_name",device,"/dev/i2c-0");
    pn.param<bool>("use_compass", use_compass,true);


    ROS_INFO("setting up i2c_client...");
    cereal::I2Ckernel i2c;

    i2c._open(device.c_str());

    ROS_INFO("setting up MPU60X0...");
    accelgyro = new MPU60X0(i2c);


    // verify connection //TODO: break if unsuccessful
    ROS_INFO("Testing device connections...");
    ROS_INFO(accelgyro->testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    ROS_INFO("Initialize MPU_6050...");
    accelgyro->initialize();
    accelgyro->setI2CMasterModeEnabled(0);
    accelgyro->setI2CBypassEnabled(1);
    accelgyro->setFullScaleGyroRange(MPU60X0_GYRO_FS_250);//set gyro scale
    accelgyro->setFullScaleAccelRange(MPU60X0_ACCEL_FS_2); //set accel scale
    ros::Duration(0.005).sleep();

    HMC58X3 magn(i2c);
    if (use_compass){
        ROS_INFO("Initialize HMC5883L...");
        // init HMC5843
        magn.init(false); // Don't set mode yet, we'll do that later on.
        // Calibrate HMC using self test, not recommended to change the gain after calibration.
        magn.calibrate(1,32); // Use gain 1=default, valid 0-7, 7 not recommended.
        // Single mode conversion was used in calibration, now set continuous mode
        magn.setMode(0);
        ros::Duration(0.010).sleep();
        magn.setDOR(0b110);
    }



    if(calibrate){
        zeroGyro();
    }

    ros::Publisher imu_pub = pn.advertise<sensor_msgs::Imu>("data", 10);
    ros::Publisher mag_pub = pn.advertise<geometry_msgs::Vector3Stamped>("mag", 10);
    imu_calib_pub = pn.advertise<std_msgs::Bool>("is_calibrated", 10 , true);

    //publish calibration status
    std_msgs::Bool calibrated;
    calibrated.data=calibrate;
    imu_calib_pub.publish(calibrated);

    //calibration service
    ros::ServiceServer service = pn.advertiseService("calibrate", calibrate_gyro);


    ros::Rate r(frequency);

    while(ros::ok())
    {
        ros::Time now = ros::Time::now();

        sensor_msgs::Imu imu_msg;
        geometry_msgs::Vector3Stamped mag_msg;

        imu_msg.header.stamp = now;
        imu_msg.header.frame_id = MPU_FRAMEID;
        mag_msg.header.stamp=now;
        mag_msg.header.frame_id = MPU_FRAMEID;


        int16_t ax, ay, az;
        int16_t gx, gy, gz;
        float mx, my, mz;

        accelgyro->getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        if (use_compass){
            magn.getValues(&mx, &my, &mz);
        }


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

        /*
        gx_f=((float) gx / 16.4f; // for degrees/s 2000 scale
        gy_f=((float) gy / 16.4f; // for degrees/s 2000 scale
        gz_f=((float) gz / 16.4f; // for degrees/s 2000 scale
        */


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


        //TODO: raw values for now , change this to correct value
        mag_msg.vector.x=mx; // divide by 10000000.0; //miligauss to Tesla ?
        mag_msg.vector.y=my;
        mag_msg.vector.z=mz;


        imu_pub.publish(imu_msg);
        mag_pub.publish(mag_msg);

        ros::spinOnce();
        r.sleep();
    }





    return 0;
}

