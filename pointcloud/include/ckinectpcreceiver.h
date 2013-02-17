#ifndef CKINECTPCRECEIVER_H
#define CKINECTPCRECEIVER_H

// ros
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

// pcl
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include <pcl_ros/transforms.h>

//#include <pcfilter.h>

#include <QMutex>
#include <string>

#include "qnode.hpp"
using namespace std;


// Useful macros
#define FPS_CALC(_WHAT_) \
    do \
{ \
    static unsigned count = 0;\
    static double last = pcl::getTime ();\
    double now = pcl::getTime (); \
    ++count; \
    if (now - last >= 1.0) \
{ \
    ROS_INFO("FPS of %s: %f", _WHAT_, double(count)/double(now - last)); \
    count = 0; \
    last = now; \
    } \
    }while(false)

template<typename PointT>
class CKinectPCReceiver : public QNode
{    
public:
    typedef pcl::PointCloud<PointT> Cloud;
    typedef typename Cloud::Ptr CloudPtr;
    typedef typename Cloud::ConstPtr CloudConstPtr;

public:
    CKinectPCReceiver(int argc, char** argv, string _nodeName, string _subTopic);
    virtual ~CKinectPCReceiver() {}
    void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input);
    void run();
    void ros_comms_init();

protected:
    void processing();

private:
    string nodeName;
    string subTopic;
//    PCFilter<PointT> filter;
    ros::Subscriber sub;

public:
    CloudPtr pCloudIn, pCloudSampled, pCloudOut, pCloud;
    CloudConstPtr pCloudConst;
    int isStarted;
    bool isUsing;

};
#include "ckinectpcreceiver.hpp"
#endif // CKINECTPCRECEIVER_H
