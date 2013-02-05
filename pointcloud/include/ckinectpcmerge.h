#ifndef CKINECTPCMERGE_H
#define CKINECTPCMERGE_H

#include <ros/ros.h>

#include "ckinectpcreceiver.h"
#include "qnode.hpp"
#include <string>
#include "pcfilter.h"

using namespace std;

typedef pcl::PointXYZRGB PointT;
typedef CKinectPCReceiver<PointT> PCReceiver;

class CKinectPCMerge : public QNode
{
public:
    typedef pcl::PointCloud<PointT> Cloud;
    typedef Cloud::Ptr CloudPtr;
    typedef Cloud::ConstPtr CloudConstPtr;

public:
    CKinectPCMerge(int _argc, char** _argv, int _numPC, vector<string> _subTopics, string _pubTopic, vector<double> _ws);
    void run();
    void ros_comms_init();

private:
    void infoDisplay();

private:
    vector<PCReceiver*> pcReceivers;
    PCFilter<PointT> filter;
    ros::Publisher pub;
    QMutex mutex;
    sensor_msgs::PointCloud2 output;
    tf::TransformListener *tf_listener;
    int argc;
    char** argv;


    int numPC;
    vector<string> subTopics;
    string pubTopic;
    vector<double> ws;
    CloudPtr pCloudOut, pCloudSampled, pCloudTransformed;
};

#endif // CKINECTPCMERGE_H
