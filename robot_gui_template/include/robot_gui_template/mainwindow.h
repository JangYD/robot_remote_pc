#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#define M_PI 3.14159265358979323846

#include <QMainWindow>
#include <QPixmap>
#include <QLabel>
#include <QImage>
#include <QTime>
#include <QtConcurrent/QtConcurrent>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <chrono>
#include <vector>

#define MAX_LIN_VEL 2.0
#define MAX_ANG_VEL 2.0

#define LIN_VEL_STEP_SIZE 0.2
#define ANG_VEL_STEP_SIZE 0.2

#define wheel_radius 3.25 //cm
#define gear_ratio 30.0

#define TICK2RAD 0.0001 // (( 360 / 60000 ) * pi)/180
                        //         pulse/rev

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void img_callback(const std_msgs::UInt8MultiArray::ConstPtr& array);
    void left_encoder_callback(const std_msgs::Int32::ConstPtr& msg);
    void right_encoder_callback(const std_msgs::Int32::ConstPtr& msg);
    void imu_callback(const sensor_msgs::Imu::ConstPtr& msg);

    void InitGraph();
    void SetGraph();
    double GraphTImer();
    double left_pulse_to_velocity(int pulse);
    double right_pulse_to_velocity(int pulse);

    double check_vel(double input, double low, double high);
    double check_linear_limit_vel(double vel);
    double check_angular_limit_vel(double vel);

public slots:
    void spinOnce();
    void updateGraph();

private slots:
    void on_up_push_btn_clicked();

    void on_down_push_btn_clicked();

    void on_left_push_btn_clicked();

    void on_right_push_btn_clicked();

    void on_null_btn_clicked();

    void on_start_btn_clicked();

    void on_stop_btn_clicked();

    void on_reset_btn_clicked();

private:
    Ui::MainWindow *ui;

    QTimer *ros_timer;
    ros::NodeHandlePtr nh_;
    ros::Subscriber imu_sub;
    ros::Subscriber img_sub;
    ros::Subscriber left_encoder_sub;
    ros::Subscriber right_encoder_sub;
    ros::Publisher button_pub;
    geometry_msgs::Twist twist;
    geometry_msgs::Twist graph_twist;

    bool GraphThread;

    double target_linear_vel;
    double target_angular_vel;

    double motor_left_velocity;
    double motor_right_velocity;

    int left_encoder_msg;
    int right_encoder_msg;

    int left_prev_pulse;
    int right_prev_pulse;

    double imu_test;

    int button_status;
};

#endif // MAINWINDOW_H
