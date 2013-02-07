
#include <ros/ros.h>

#include "mainwindow.h"
#include <QtGui>

int main (int argc, char** argv)
{
    ros::init (argc, argv, "dhri_calibrationOneKinect");

    QApplication a(argc, argv);
    MainWindow view;
    view.path = QString(argv[1]);
    view.fileName = QString(argv[2]);

    view.show();

    return a.exec();
}
