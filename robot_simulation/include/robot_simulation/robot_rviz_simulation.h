#ifndef ROBOT_RVIZ_SIMULATION_H
#define ROBOT_RVIZ_SIMULATION_H

#define LEFT 0
#define RIGHT 1
#define TICK2RAD 0.0001
#define wheel_radius 3.25 //m
#define robot_thread 0.181
#define M_PI 3.14159265358979323846
#define d2r (M_PI /180.0)


#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>

class robot_rviz_simulation
{
  public:
    robot_rviz_simulation();
    ~robot_rviz_simulation();
    void commandVelocityCallback(const geometry_msgs::TwistConstPtr cmd_vel_msg);
    void ahrs_yaw_data_callback(const std_msgs::Float64::ConstPtr& ahrs_msg);
    bool update_odometry(ros::Duration diff_time);
    void left_encoder_callback(const std_msgs::Int32::ConstPtr& msg);
    void right_encoder_callback(const std_msgs::Int32::ConstPtr& msg);

    void update_joint(void);
    void update_tf(geometry_msgs::TransformStamped& odom_tf);
    bool update();
    double left_pulse_to_velocity(int pulse);
    double right_pulse_to_velocity(int pulse);

  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_priv_;

    ros::Time last_cmd_vel_time_;
    ros::Time prev_update_time_;

    ros::Publisher joint_states_pub_;
    ros::Publisher odom_pub_;

    ros::Subscriber left_encoder_sub;
    ros::Subscriber right_encoder_sub;

    ros::Subscriber cmd_vel_sub_;
    ros::Subscriber ahrs_sub;

    sensor_msgs::JointState joint_states_;
    nav_msgs::Odometry odom_;
    tf::TransformBroadcaster tf_broadcaster_;

    double wheel_speed_cmd_[2];
    double goal_linear_velocity_;
    double goal_angular_velocity_;
    double cmd_vel_timeout_;

    double odom_pose_[3];
    double odom_vel_[3];
    double pose_cov_[36];

    std::string joint_states_name_[2];

    double last_position_[2];
    double last_velocity_[2];

    int left_encoder_msg;
    int right_encoder_msg;

    double motor_velocity[2];

    int left_pulse_prev;
    int right_pulse_prev;

    double ahrs_yaw;
};

#endif // ROBOT_RVIZ_SIMULATION_H
