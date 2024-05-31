#include "mainwindow.h"
#include <QApplication>
#include <QFuture>



int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_gui_node", ros::init_options::AnonymousName);
//  ros::init(argc, argv, "robot_gui_node");
  QApplication a(argc, argv);
  MainWindow w;
  w.show();

  return a.exec();
}
