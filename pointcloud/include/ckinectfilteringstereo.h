#ifndef CKINECTFILTERINGSTEREO_H
#define CKINECTFILTERINGSTEREO_H


#include <pcfilter.h>
#include <nav_msgs/GridCells.h>

typedef pcl::PointXYZRGB PointT;

class CKinectFilteringStereo
{
public:
    typedef pcl::PointXYZI PointID;
    typedef pcl::PointCloud<PointID> CloudID;
    typedef CloudID::Ptr CloudPtrID;
    typedef CloudID::ConstPtr CloudConstPtrID;

public:
    CKinectFilteringStereo();


private:
    PCFilter<PointT> filter;

};

#endif // CKINECTFILTERINGSTEREO_H
