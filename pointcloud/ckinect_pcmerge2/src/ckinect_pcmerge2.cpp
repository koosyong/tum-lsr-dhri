
#include <ros/ros.h>

#include "ckinectpcmerge.h"
#include "mainwindow.h"
#include <QtGui>

bool isWorkspace = true;

int main (int argc, char** argv)
{
//    ros::init (argc, argv, "ckinect_calibration_stereo");
    QApplication a(argc, argv);

    vector<double> ws;
    int numSubTopics;
    vector<string> subTopics;
    string pubTopic;

    numSubTopics = 2;
    subTopics.push_back("/camera1/depth_registered/points");
    subTopics.push_back("/camera2/depth_registered/points");
    pubTopic = "/ckinect/points/merged";
    ws.push_back(0.0);
    ws.push_back(0.0);
    ws.push_back(0.0);
    ws.push_back(2.0);
    ws.push_back(2.0);
    ws.push_back(2.0);

    CKinectPCMerge pcmerge(argc, argv, numSubTopics, subTopics, pubTopic, ws);
    pcmerge.on_init();
    MainWindow view;

    if(isWorkspace) view.show();
    else view.run();

    return a.exec();
}
