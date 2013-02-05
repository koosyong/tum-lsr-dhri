#ifndef CKINECTFILTERING_H
#define CKINECTFILTERING_H

#include <ckinect.h>
#include <pcfilter.h>
#include <nav_msgs/GridCells.h>

typedef pcl::PointXYZRGB PointT;

class CKinectFiltering : public CKinect<PointT>
{
public:
    typedef pcl::PointXYZI PointID;
    typedef pcl::PointCloud<PointID> CloudID;
    typedef CloudID::Ptr CloudPtrID;
    typedef CloudID::ConstPtr CloudConstPtrID;

public:
    CKinectFiltering();
    void run();

private:
    PCFilter<PointT> filter;

    ros::Publisher pub_filtered;
    ros::Publisher pub_segmentedRGB;
    ros::Publisher pub_segmentedID;
    ros::Publisher pub_grid;    
    nav_msgs::GridCells workspace;

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
