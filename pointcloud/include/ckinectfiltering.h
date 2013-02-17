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
    bool isParam;
    int param_sub_num;
    vector<string> param_sub_topics;
    vector<string> param_sub_frames;
    string param_pub_topic;
    string param_pub_frame;

    bool param_workspace_on;
    double param_workspace_x;
    double param_workspace_y;
    double param_workspace_z;
    double param_workspace_width;
    double param_workspace_height;
    double param_workspace_zheight;
    string param_workspace_topic;

    bool param_downsampling_on;
    double param_downsampling_leaf;

    bool param_planeExtraction_on;
    int param_planeExtraction_numPlane;

    bool param_segmentation_on;
    double param_segmentation_tolerance;
    double param_segmentation_minSize;
    double param_segmentation_maxSize;
    string param_segmentation_topicRGB;
    string param_segmentation_topicID;

private:
    void readParam();
    void processing();
    void infoDisplay();
};

#endif // CKINECTFILTERING_H
