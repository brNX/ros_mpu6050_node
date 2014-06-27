#include "mpu6050nodelet.h"

PLUGINLIB_DECLARE_CLASS(mpu_6050, MPU6050Nodelet, MPU6050Nodelet, nodelet::Nodelet)

void MPU6050Nodelet::onInit()
{
  NODELET_INFO("Initializing MPU6050 Nodelet");

  // TODO: Do we want the single threaded or multithreaded NH?
  ros::NodeHandle nh         = getNodeHandle();
  ros::NodeHandle pnh = getPrivateNodeHandle();

  mpu_ = new MPU6050(nh, pnh);
}
