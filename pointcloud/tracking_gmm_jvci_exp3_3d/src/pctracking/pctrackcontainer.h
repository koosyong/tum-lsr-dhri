#ifndef PCTRACKCONTAINER_H
#define PCTRACKCONTAINER_H

// pcl
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include <pcl_ros/transforms.h>


//#include "pctrack.h"
#include "pcobject.h"
#include "trackcontainer.h"
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

class PCTrackContainer : public TrackContainer<PCObject>
{
    typedef pcl::PointXYZRGB PointT;
    typedef pcl::PointCloud<PointT> Cloud;
    typedef Cloud::Ptr CloudPtr;
    typedef Cloud::ConstPtr CloudConstPtr;

public:
    PCTrackContainer(int _maxFrame = 1000);

public:
    void toPointCloudXYZI(Cloud &cloudOut);
    visualization_msgs::MarkerArray toMarkerGMMs();
    visualization_msgs::Marker toMarkerEdges();

    void evaluate();

public:
    int numTruePoints;
    int numFalsePoints;
    int numTotalPoints;

private:
    inline float SIGN(float x) {return (x >= 0.0f) ? +1.0f : -1.0f;}
    inline float NORM(float a, float b, float c, float d) {return sqrt(a * a + b * b + c * c + d * d);}


private:
    int maxFrame;
    int oldCnt;

};

#endif // PCTRACKCONTAINER_H
