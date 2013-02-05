
#include <ros/ros.h>

#include "mainwindow.h"
#include <QtGui>

bool isCalibration = true;

int main (int argc, char** argv)
{
    ros::init (argc, argv, "dhri_calibrationOneKinect");

    QApplication a(argc, argv);
    MainWindow view;

    if(isCalibration) view.show();
    else view.pubTFCalib();

    return a.exec();
}
