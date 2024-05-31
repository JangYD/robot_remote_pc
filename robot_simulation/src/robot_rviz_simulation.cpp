#include "robot_rviz_simulation.h"

robot_rviz_simulation::robot_rviz_simulation()
: nh_priv_("~")
{
  nh_.param("wheel_left_joint_name", joint_states_name_[LEFT],  std::string("left_wheel_joint"));
  nh_.param("wheel_right_joint_name", joint_states_name_[RIGHT],  std::string("right_wheel_joint"));
  nh_.param("joint_states_frame", joint_states_.header.frame_id, std::string("base_footprint"));
  nh_.param("odom_frame", odom_.header.frame_id, std::string("odom"));
  nh_.param("base_frame", odom_.child_frame_id, std::string("base_footprint"));
  left_encoder_sub = nh_.subscribe<std_msgs::Int32>("/left_encoder_pub", 1000, &robot_rviz_simulation::left_encoder_callback, this);
  right_encoder_sub = nh_.subscribe<std_msgs::Int32>("/right_encoder_pub", 1000, &robot_rviz_simulation::right_encoder_callback, this);

  wheel_speed_cmd_[LEFT]  = 0.0;
  wheel_speed_cmd_[RIGHT] = 0.0;
  goal_linear_velocity_   = 0.0;
  goal_angular_velocity_  = 0.0;
  cmd_vel_timeout_        = 1.0;
  last_position_[LEFT]    = 0.0;
  last_position_[RIGHT]   = 0.0;
  last_velocity_[LEFT]    = 0.0;
  last_velocity_[RIGHT]   = 0.0;

  odom_pose_[0] = 0.0;
  odom_pose_[1] = 0.0;
  odom_pose_[2] = 0.0;

  odom_vel_[0] = 0.0;
  odom_vel_[1] = 0.0;
  odom_vel_[2] = 0.0;

  joint_states_.name.push_back(joint_states_name_[LEFT]);
  joint_states_.name.push_back(joint_states_name_[RIGHT]);
  joint_states_.position.resize(2,0.0);
  joint_states_.velocity.resize(2,0.0);
  joint_states_.effort.resize(2,0.0);
  joint_states_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 100);
  odom_pub_         = nh_.advertise<nav_msgs::Odometry>("odom", 100);

  cmd_vel_sub_  = nh_.subscribe("cmd_vel", 100,  &robot_rviz_simulation::commandVelocityCallback, this);
  ahrs_sub = nh_.subscribe("/imu/yaw", 5, &robot_rviz_simulation::ahrs_yaw_data_callback, this);

  left_pulse_prev = 0;
  right_pulse_prev = 0;

  prev_update_time_ = ros::Time::now();
}

robot_rviz_simulation::~robot_rviz_simulation()
{
}

void robot_rviz_simulation::commandVelocityCallback(const geometry_msgs::TwistConstPtr cmd_vel_msg)
{
  last_cmd_vel_time_ = ros::Time::now();

  goal_linear_velocity_  = cmd_vel_msg->linear.x;
  goal_angular_velocity_ = cmd_vel_msg->angular.z;
}

void robot_rviz_simulation::left_encoder_callback(const std_msgs::Int32::ConstPtr& msg)
{
  left_encoder_msg = msg->data;
  motor_velocity[LEFT] = left_pulse_to_velocity(left_encoder_msg);
}

void robot_rviz_simulation::right_encoder_callback(const std_msgs::Int32::ConstPtr& msg)
{
  right_encoder_msg = msg->data;
  motor_velocity[RIGHT] = right_pulse_to_velocity(right_encoder_msg);
}

void robot_rviz_simulation::ahrs_yaw_data_callback(const std_msgs::Float64::ConstPtr& ahrs_msg)
{
  ahrs_yaw = ahrs_msg->data;
}


double robot_rviz_simulation::left_pulse_to_velocity(int pulse)
{
  int left_pulse_diff = pulse - left_pulse_prev;

  double left_rad = left_pulse_diff * TICK2RAD;

  double velocity = left_rad * wheel_radius;

  left_pulse_prev = pulse;

  return velocity;
}

double robot_rviz_simulation::right_pulse_to_velocity(int pulse)
{
  int right_pulse_diff = pulse - right_pulse_prev;

  double left_rad = right_pulse_diff * TICK2RAD;

  double velocity = left_rad * wheel_radius;

  right_pulse_prev = pulse;

  return velocity;
}


bool robot_rviz_simulation::update_odometry(ros::Duration diff_time)
{
  double wheel_l, wheel_r;
  double delta_s, delta_theta;
  double v[2], w[2];

  wheel_l = wheel_r     = 0.0;
  delta_s = delta_theta = 0.0;

  v[LEFT]  = motor_velocity[LEFT];
  w[LEFT]  = v[LEFT] / wheel_radius;
  v[RIGHT] = motor_velocity[RIGHT];
  w[RIGHT] = v[RIGHT] / wheel_radius;

  last_velocity_[LEFT]  = w[LEFT];
  last_velocity_[RIGHT] = w[RIGHT];

  wheel_l = w[LEFT]  * diff_time.toSec();
  wheel_r = w[RIGHT] * diff_time.toSec();

  if(isnan(wheel_l))
  {
    wheel_l = 0.0;
  }

  if(isnan(wheel_r))
  {
    wheel_r = 0.0;
  }

  last_position_[LEFT]  += wheel_l;
  last_position_[RIGHT] += wheel_r;

  delta_s     = wheel_radius * (wheel_r + wheel_l) / 2.0;
  delta_theta = wheel_radius * (wheel_r - wheel_l) / robot_thread;

  odom_pose_[0] -= delta_s * cos(odom_pose_[2] + (delta_theta / 2.0));
  odom_pose_[1] -= delta_s * sin(odom_pose_[2] + (delta_theta / 2.0));
  odom_pose_[2] -= delta_theta;

  odom_vel_[0] = delta_s / diff_time.toSec();
  odom_vel_[1] = 0.0;
  odom_vel_[2] = delta_theta / diff_time.toSec();

  odom_.pose.pose.position.x = odom_pose_[0];
  odom_.pose.pose.position.y = odom_pose_[1];
  odom_.pose.pose.position.z = 0;
  odom_.pose.pose.orientation = tf::createQuaternionMsgFromYaw(odom_pose_[2]);

  odom_.twist.twist.linear.x  = goal_linear_velocity_;
  odom_.twist.twist.angular.z = goal_angular_velocity_;

  return true;
}

void robot_rviz_simulation::update_joint(void)
{
  joint_states_.position[LEFT]  = last_position_[LEFT];
  joint_states_.position[RIGHT] = last_position_[RIGHT];
  joint_states_.velocity[LEFT]  = last_velocity_[LEFT];
  joint_states_.velocity[RIGHT] = last_velocity_[RIGHT];
}

void robot_rviz_simulation::update_tf(geometry_msgs::TransformStamped& odom_tf)
{
  odom_tf.header = odom_.header;
  odom_tf.child_frame_id = odom_.child_frame_id;
  odom_tf.transform.translation.x = odom_.pose.pose.position.x;
  odom_tf.transform.translation.y = odom_.pose.pose.position.y;
  odom_tf.transform.translation.z = odom_.pose.pose.position.z;
  odom_tf.transform.rotation = odom_.pose.pose.orientation;
}

bool robot_rviz_simulation::update()
{
  ros::Time time_now = ros::Time::now();
  ros::Duration step_time = time_now - prev_update_time_;
  prev_update_time_ = time_now;

  update_odometry(step_time);
  odom_.header.stamp = time_now;
  odom_pub_.publish(odom_);

  update_joint();
  joint_states_.header.stamp = time_now;
  joint_states_pub_.publish(joint_states_);

  geometry_msgs::TransformStamped odom_tf;
  update_tf(odom_tf);
  tf_broadcaster_.sendTransform(odom_tf);

  return true;
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "robot_simulaion_node");
  robot_rviz_simulation robot_sim;

  ros::Rate loop_rate(30);

  while (ros::ok())
  {
    robot_sim.update();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
