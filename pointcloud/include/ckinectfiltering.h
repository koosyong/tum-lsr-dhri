#ifndef CKINECTFILTERING_H
#define CKINECTFILTERING_H

#include "ckinectpcreceiver.h"
#include <pcfilter.h>
#include <nav_msgs/GridCells.h>

using namespace std;

typedef pcl::PointXYZRGB PointT;
typedef CKinectPCReceiver<PointT> PCReceiver;

class CKinectFiltering : public QNode
{
public:
    typedef pcl::PointCloud<PointT> Cloud;
    typedef Cloud::Ptr CloudPtr;
    typedef Cloud::ConstPtr CloudConstPtr;

    typedef pcl::PointXYZI PointID;
    typedef pcl::PointCloud<PointID> CloudID;
    typedef CloudID::Ptr CloudPtrID;
    typedef CloudID::ConstPtr CloudConstPtrID;       

public:
    CKinectFiltering();
    void run();
    void ros_comms_init();

private:
    vector<PCReceiver*> pcReceivers;
    PCFilter<PointT> filter;

    ros::Publisher pub_filtered;
    ros::Publisher pub_segmentedRGB;
    ros::Publisher pub_segmentedID;
    ros::Publisher pub_grid;    
    nav_msgs::GridCells workspace;
    QMutex mutex;
    sensor_msgs::PointCloud2 output;
    tf::TransformListener *tf_listener;

    // param
    string paramValue_pubTopic_segmentedRGB;
    string paramValue_pubTopic_segmentedID;
    string paramValue_pubTopic_workspace;

    bool paramValue_workspace_on;
    double paramValue_workspace_x;
    double paramValue_workspace_y;
    double paramValue_workspace_z;
    double paramValue_workspace_height;
    double paramValue_workspace_width;
    double paramValue_workspace_zheight;
    bool paramValue_downsampling_on;
    double paramValue_downsampling_leaf;
    bool paramValue_planeExtraction_on;
    int paramValue_planeExtraction_numPlane;
    bool paramValue_segmentation_on;
    double paramValue_segmentation_tolerance;
    double paramValue_segmentation_minSize;
    double paramValue_segmentation_maxSize;

protected:
    void init();
    void processing();
};

#endif // CKINECTFILTERING_H
