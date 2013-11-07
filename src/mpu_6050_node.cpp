#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include "i2c_ros/i2c.h"

#define MPU_FRAMEID "/base_mpu_6050"

bool calibrate;
ros::Publisher imu_calib_pub;
ros::ServiceClient * clientptr;

extern "C"{

#include "mpu9150.h"
#include "local_defaults.h"


int i2c_write(unsigned char slave_addr, unsigned char reg_addr,unsigned char length, unsigned char const *data){

    i2c_ros::i2c srv;

    srv.request.address=slave_addr;
    srv.request.operation=i2c_ros::i2cRequest::WRITE;
    srv.request.size=length;
    srv.request.reg=reg_addr;

    srv.request.size=length;
    for(int i =0;i<length;i++){
        srv.request.data.push_back(data[i]);
    }

    if (!clientptr->call(srv)){
        ROS_ERROR("Failed to call service i2c_ros");
        return -1;
    }

    return 0;
}

int i2c_read(unsigned char slave_addr, unsigned char reg_addr,unsigned char length, unsigned char *data){

    i2c_ros::i2c srv;

    srv.request.address=slave_addr;
    srv.request.reg=reg_addr;
    srv.request.operation=i2c_ros::i2cRequest::READ;
    srv.request.size=length;

    if (clientptr->call(srv)){

        if(srv.response.data.size()>0){
            for(int i =0;i<srv.response.data.size();i++){
                data[i]=srv.response.data[i];
            }
        }else{
            ROS_WARN("MPU6050 - %s - read response data empty", __FUNCTION__);
        }

    }
    else{
        ROS_ERROR("Failed to call service i2c_ros");
        return -1;
    }

    return 0;
}
}


// calibration parameters
//int16_t gyro_off_x=0, gyro_off_y=0, gyro_off_z=0;
//float acc_scale_x, acc_scale_y, acc_scale_z, magn_scale_x, magn_scale_y, magn_scale_z;

/**
 * Computes gyro offsets
*/
/*void zeroGyro() {
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
}*/

bool calibrate_gyro(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp){
    /*zeroGyro();*/

    //publish calibration status
    calibrate=true;
    std_msgs::Bool calibrated;
    calibrated.data=calibrate;
    imu_calib_pub.publish(calibrated);


    return true;
}


int main(int argc, char **argv){

    ros::init(argc, argv, "mpu_6050_node");

    ros::NodeHandle pn("~");
    ros::NodeHandle n;


    ROS_INFO("Starting mpu_6050_node...");
    ROS_INFO("setting up i2c_operation client...");
    ros::ServiceClient client = n.serviceClient<i2c_ros::i2c>("i2c_operation");
    clientptr = &client;


    /****
     *IMU parameters
     ***/
    int sample_rate;
    pn.param<int>("frequency", sample_rate ,DEFAULT_SAMPLE_RATE_HZ);
    pn.param<bool>("autocalibrate",calibrate,true);
    //pn.param<bool>("use_compass", use_compass,true);


    ROS_INFO("setting up MPU60X0...");

    //TODO:  change this to a parameter to use compass
    int yaw_mix_factor = DEFAULT_YAW_MIX_FACTOR;
    mpudata_t mpu;

    //mpu9150_set_debug(1);
    ROS_INFO("Initialize MPU_6050...");
    if (mpu9150_init(sample_rate, 0)){
        ROS_FATAL("MPU6050 - %s - MPU6050 connection failed",__FUNCTION__);
        ROS_BREAK();
    }
    memset(&mpu, 0, sizeof(mpudata_t));
    if (sample_rate == 0)
        ROS_BREAK();


    ros::Publisher imu_pub = pn.advertise<sensor_msgs::Imu>("imu/data", 10);
    imu_calib_pub = pn.advertise<std_msgs::Bool>("imu/is_calibrated", 10 , true);

    //publish calibration status
    std_msgs::Bool calibrated;
    calibrated.data=calibrate;
    imu_calib_pub.publish(calibrated);

    //calibration service
    ros::ServiceServer service = pn.advertiseService("imu/calibrate", calibrate_gyro);


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


             //TODO: needs conversion

             imu_msg.linear_acceleration.x=mpu.calibratedAccel[0];
             imu_msg.linear_acceleration.y=mpu.calibratedAccel[1];
             imu_msg.linear_acceleration.z=mpu.calibratedAccel[2];

             imu_msg.angular_velocity.x=mpu.rawGyro[0];
             imu_msg.angular_velocity.y=mpu.rawGyro[1];
             imu_msg.angular_velocity.z=mpu.rawGyro[2];

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

