#ifndef FLAT_WORLD_IMU_NODE_H
#define FLAT_WORLD_IMU_NODE_H

#define GRAVITY 9.8

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

class flat_world_imu_node
{
public:
  flat_world_imu_node();
  bool init();

private:
  ros::NodeHandle nh;
  ros::Publisher pub;
  ros::Subscriber sub;
  ros::Time last_published_time;
  void msg_call_back(const sensor_msgs::Imu::ConstPtr imu_in);

};

#endif // FLAT_WORLD_IMU_NODE_H
