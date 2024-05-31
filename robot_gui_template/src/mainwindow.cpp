#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <ros/ros.h>
#include <qcustomplot/qcustomplot.h>

#include <QtCore>
#include <QtGui>
#include <QtWidgets>

using namespace std;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    nh_.reset(new ros::NodeHandle("~"));

    ros_timer = new QTimer(this);
    connect(ros_timer, SIGNAL(timeout()), this, SLOT(spinOnce()));
    InitGraph();
    ros_timer->start(1);

    imu_sub = nh_->subscribe<sensor_msgs::Imu>("/imu/data", 10, &MainWindow::imu_callback, this);

    std::string img_topic;
    nh_->param<std::string>("img_topic", img_topic, "/camera/image");
    img_sub  = nh_->subscribe<std_msgs::UInt8MultiArray>(img_topic, 1, &MainWindow::img_callback, this);

    std::string button_topic;
    nh_->param<std::string>("button_topic", button_topic, "/cmd_vel");

    left_encoder_sub = nh_->subscribe<std_msgs::Int32>("/left_encoder_pub", 1000, &MainWindow::left_encoder_callback, this);
    right_encoder_sub = nh_->subscribe<std_msgs::Int32>("/right_encoder_pub", 1000, &MainWindow::right_encoder_callback, this);;

    button_pub = nh_->advertise<geometry_msgs::Twist>(button_topic, 10);
    target_linear_vel = 0.0;
    target_angular_vel= 0.0;
    left_prev_pulse = 0;
    right_prev_pulse = 0;
}

MainWindow::~MainWindow()
{
  delete ros_timer;
  delete ui;
}

void MainWindow::spinOnce()
{
  if(ros::ok())
  {

    ros::spinOnce();
  }
  else
  {
    QApplication::quit();
  }
}

void MainWindow::img_callback(const std_msgs::UInt8MultiArray::ConstPtr& array)
{
  cv::Mat frame = cv::imdecode(cv::Mat(array->data),1);

  QImage qimg(frame.data, frame.cols, frame.rows, frame.step, QImage::Format_RGB888);
  qimg = qimg.rgbSwapped();

  QPixmap pixmap = QPixmap::fromImage(qimg).scaled(ui->camera_lab->size(), Qt::KeepAspectRatio);
  ui->camera_lab->setPixmap(pixmap);
}

void MainWindow::imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
   imu_test = msg->angular_velocity.x;
}

void MainWindow::left_encoder_callback(const std_msgs::Int32::ConstPtr& msg)
{
  left_encoder_msg = msg->data;
  motor_left_velocity = left_pulse_to_velocity(left_encoder_msg);
}

void MainWindow::right_encoder_callback(const std_msgs::Int32::ConstPtr& msg)
{
  right_encoder_msg = msg->data;
  motor_right_velocity = right_pulse_to_velocity(right_encoder_msg);
}

double MainWindow::GraphTImer()
{
  static QTime time(QTime::currentTime());
  double setTime = time.elapsed()/1000.0;

  return setTime;
}

double MainWindow::left_pulse_to_velocity(int pulse)
{
  double pulse_diff = pulse - left_prev_pulse;

  double left_rad = TICK2RAD * pulse_diff;

  double current_velocity = left_rad * wheel_radius;

  constexpr int num_samples = 10;
  static std::vector<double> velocity_samples;
  velocity_samples.push_back(current_velocity);

  if (velocity_samples.size() > num_samples)
  {
    velocity_samples.erase(velocity_samples.begin());
  }

  double sum_velocity = 0.0;
  for (const auto& vel : velocity_samples)
  {
    sum_velocity += vel;
  }

  double average_velocity = sum_velocity / velocity_samples.size();

  left_prev_pulse = pulse;

  return average_velocity;
}

double MainWindow::right_pulse_to_velocity(int pulse)
{
  double pulse_diff = pulse - right_prev_pulse;

  double right_rad = TICK2RAD * pulse_diff;

  double current_velocity = right_rad * wheel_radius;

  constexpr int num_samples = 10;
  static std::vector<double> velocity_samples;
  velocity_samples.push_back(current_velocity);

  if (velocity_samples.size() > num_samples)
  {
    velocity_samples.erase(velocity_samples.begin());
  }

  double sum_velocity = 0.0;
  for (const auto& vel : velocity_samples)
  {
    sum_velocity += vel;
  }

  double average_velocity = sum_velocity / velocity_samples.size();

  right_prev_pulse = pulse;

  return average_velocity;
}

double MainWindow::check_vel(double input, double low, double high)
{
  if(input < low)
  {
    input = low;
  }
  else if(input > high)
  {
    input = high;
  }
  return input;
}

double MainWindow::check_linear_limit_vel(double vel)
{
  vel = check_vel(vel, -MAX_LIN_VEL, MAX_LIN_VEL);
  return vel;
}

double MainWindow::check_angular_limit_vel(double vel)
{
  vel = check_vel(vel, -MAX_ANG_VEL, MAX_ANG_VEL);
  return vel;
}

void MainWindow::on_up_push_btn_clicked()
{
  target_linear_vel = check_linear_limit_vel(target_linear_vel + LIN_VEL_STEP_SIZE);

  twist.linear.x = target_linear_vel;
  twist.linear.y = 0;
  twist.linear.z = 0;

  twist.angular.x = 0.0;
  twist.angular.y = 0.0;
  twist.angular.z = 0.0;

  button_pub.publish(twist);
}

void MainWindow::on_down_push_btn_clicked()
{
  target_linear_vel = check_linear_limit_vel(target_linear_vel - LIN_VEL_STEP_SIZE);

  twist.linear.x = target_linear_vel;
  twist.linear.y = 0;
  twist.linear.z = 0;

  twist.angular.x = 0.0;
  twist.angular.y = 0.0;
  twist.angular.z = 0.0;

  button_pub.publish(twist);
}

void MainWindow::on_left_push_btn_clicked()
{
  target_angular_vel = check_linear_limit_vel(target_angular_vel + ANG_VEL_STEP_SIZE);

  twist.linear.x = 0;
  twist.linear.y = 0;
  twist.linear.z = 0;

  twist.angular.x = 0.0;
  twist.angular.y = 0.0;
  twist.angular.z = target_angular_vel;

  button_pub.publish(twist);
}

void MainWindow::on_right_push_btn_clicked()
{
  target_angular_vel = check_linear_limit_vel(target_angular_vel - ANG_VEL_STEP_SIZE);

  twist.linear.x = 0;
  twist.linear.y = 0;
  twist.linear.z = 0;

  twist.angular.x = 0.0;
  twist.angular.y = 0.0;
  twist.angular.z = target_angular_vel;

  button_pub.publish(twist);
}


void MainWindow::on_null_btn_clicked()
{
  target_linear_vel = 0.0;
  target_angular_vel = 0.0;

  twist.linear.x = target_linear_vel;
  twist.linear.y = 0;
  twist.linear.z = 0;

  twist.angular.x = 0;
  twist.angular.y = 0;
  twist.angular.z = target_angular_vel;
  button_pub.publish(twist);
}

void MainWindow::InitGraph()
{
  SetGraph();
}

void MainWindow::SetGraph()
{
  QSharedPointer<QCPAxisTickerTime> GraphTimeTicker (new QCPAxisTickerTime);
  GraphTimeTicker->setTimeFormat("%m:%s");

  ui->Motor1->legend->setVisible(true);
  ui->Motor1->xAxis->setLabel("Time");
  ui->Motor1->xAxis->setTicker(GraphTimeTicker);
  ui->Motor1->axisRect()->setupFullAxesBox();
  ui->Motor1->yAxis->setLabel("cm/s");
  ui->Motor1->yAxis->setRange(50,-50);
  ui->Motor1->addGraph();
  ui->Motor1->graph(0)->setPen(QPen(Qt::red));
  ui->Motor1->graph(0)->setName("Motor1");

  ui->Motor2->legend->setVisible(true);
  ui->Motor2->xAxis->setLabel("Time");
  ui->Motor2->xAxis->setTicker(GraphTimeTicker);
  ui->Motor2->axisRect()->setupFullAxesBox();
  ui->Motor2->yAxis->setLabel("cm/s");
  ui->Motor2->yAxis->setRange(50,-50);
  ui->Motor2->addGraph();
  ui->Motor2->graph(0)->setPen(QPen(Qt::blue));
  ui->Motor2->graph(0)->setName("Motor2");

  connect(ui->Motor1->xAxis, SIGNAL(rangeChanged(QCPRange)), ui->Motor1->xAxis2, SLOT(setRange(QCPRange)));
  connect(ui->Motor1->yAxis, SIGNAL(rangeChanged(QCPRange)), ui->Motor1->yAxis2, SLOT(setRange(QCPRange)));
  connect(ros_timer, SIGNAL(timeout()), this, SLOT(updateGraph()));

  connect(ui->Motor2->xAxis, SIGNAL(rangeChanged(QCPRange)), ui->Motor2->xAxis2,SLOT(setRange(QCPRange)));
  connect(ui->Motor2->xAxis, SIGNAL(rangeChanged(QCPRange)), ui->Motor2->yAxis2,SLOT(setRange(QCPRange)));
  connect(ros_timer, SIGNAL(timeout()), this, SLOT(updateGraph()));

  ui->Motor1->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
  ui->Motor2->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
}

void MainWindow::updateGraph()
{
  double set_graph_timer = static_cast<double>(this->GraphTImer());

  if(GraphThread == true)
  {
    ui->Motor1->graph(0)->addData(set_graph_timer, motor_left_velocity);
    ui->Motor2->graph(0)->addData(set_graph_timer, motor_right_velocity);

    ui->Motor1->xAxis->setRange(set_graph_timer+0.1, 10, Qt::AlignRight);
    ui->Motor1->yAxis->rescale(true);
    ui->Motor1->replot();
    ui->Motor1->update();

    ui->Motor2->xAxis->setRange(set_graph_timer+0.1, 10, Qt::AlignRight);
    ui->Motor2->yAxis->rescale(true);
    ui->Motor2->replot();
    ui->Motor2->update();
  }
}

void MainWindow::on_start_btn_clicked()
{
  GraphThread = true;
}

void MainWindow::on_stop_btn_clicked()
{
  GraphThread = false;
}

void MainWindow::on_reset_btn_clicked()
{
  ui->Motor1->graph(0)->data().data()->clear();
  ui->Motor1->replot();

  ui->Motor2->graph(0)->data().data()->clear();
  ui->Motor2->replot();
}
