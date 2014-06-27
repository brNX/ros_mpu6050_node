#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>
#include "MPU60X0/MPU60X0.h"
#include "HMC58X3/HMC58X3.h"
#include "mpu6050.h"

MPU6050::MPU6050(ros::NodeHandle nh, ros::NodeHandle pnh)
    :pnh_(pnh),nh_(nh),
     gyro_off_x(0), gyro_off_y(0), gyro_off_z(0)
{

    ROS_INFO("Starting mpu_6050...");


    /****
     *IMU parameters
     ***/
    int frequency;
    std::string device;
    pnh_.param<int>("frequency", frequency ,20);
    pnh_.param<bool>("autocalibrate",calibrate,true);
    pnh_.param<std::string>("device_name",device,"/dev/i2c-0");
    pnh_.param<bool>("use_compass", use_compass,true);


    ROS_INFO("setting up i2c_client...");
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

    magn = new HMC58X3(i2c);
    if (use_compass){
        ROS_INFO("Initialize HMC5883L...");
        // init HMC5843
        magn->init(false); // Don't set mode yet, we'll do that later on.
        // Calibrate HMC using self test, not recommended to change the gain after calibration.
        magn->calibrate(1,32); // Use gain 1=default, valid 0-7, 7 not recommended.
        // Single mode conversion was used in calibration, now set continuous mode
        magn->setMode(0);
        ros::Duration(0.010).sleep();
        magn->setDOR(0b110);
    }



    if(calibrate){
        zeroGyro();
    }

    imu_pub = nh_.advertise<sensor_msgs::Imu>("imu/data_raw", 10);
    mag_pub = nh_.advertise<geometry_msgs::Vector3Stamped>("imu/mag", 10);
    imu_calib_pub =  nh_.advertise<std_msgs::Bool>("imu/is_calibrated", 10 , true);

    //publish calibration status
    std_msgs::Bool calibrated;
    calibrated.data=calibrate;
    imu_calib_pub.publish(calibrated);

    //calibration service
    service =  nh_.advertiseService("imu/calibrate",&MPU6050::calibrate_gyro,this);

    //setup ros timer with the chosen frequency
    timer = nh_.createTimer(ros::Rate(frequency), &MPU6050::runOnce,this);

}


MPU6050::~MPU6050(){
    delete accelgyro;
    delete magn;
}


void MPU6050::runOnce(const ros::TimerEvent &event){
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
        magn->getValues(&mx, &my, &mz);
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
}

/**
 * Computes gyro offsets
*/
void MPU6050::zeroGyro() {
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

bool MPU6050::calibrate_gyro(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp){
    zeroGyro();

    //publish calibration status
    calibrate=true;
    std_msgs::Bool calibrated;
    calibrated.data=calibrate;
    imu_calib_pub.publish(calibrated);

    return true;
}
