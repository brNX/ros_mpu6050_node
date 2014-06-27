#ifndef MPU6050NODELET_H
#define MPU6050NODELET_H

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "mpu6050.h"

class MPU6050Nodelet : public nodelet::Nodelet
{
public:
    virtual void onInit();

private:
    MPU6050 * mpu_;
};

#endif // MPU6050NODELET_H
