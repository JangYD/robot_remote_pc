#include "flat_world_imu_node.h"

flat_world_imu_node::flat_world_imu_node()
{
  bool init_result = init();
  ROS_ASSERT(init_result);
}


bool flat_world_imu_node::init()
{
  pub = nh.advertise<sensor_msgs::Imu>("imu_out", 10);
  sub = nh.subscribe("imu_in", 150, &flat_world_imu_node::msg_call_back, this);

  return true;
}

void flat_world_imu_node::msg_call_back(const sensor_msgs::Imu::ConstPtr imu_in)
{
  if (last_published_time.isZero() || imu_in->header.stamp > last_published_time)
  {
    last_published_time = imu_in->header.stamp;
    sensor_msgs::Imu imu_out = *imu_in;
    imu_out.linear_acceleration.x = 0.0;
    imu_out.linear_acceleration.y = 0.0;
    imu_out.linear_acceleration.z = GRAVITY;
    pub.publish(imu_out);
  }
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "flat_world_imu_node");

  flat_world_imu_node flat_world_imu_node;

  ros::spin();

  return 0;
}
