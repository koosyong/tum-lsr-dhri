#ifndef PCPUBLISHER_H
#define PCPUBLISHER_H

// pcl
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include <pcl_ros/transforms.h>

#include "pcobject.h"
#include "trackcontainer.h"

#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>



class PCPublisher
{
public:
    PCPublisher();
    PCPublisher(shared_ptr< TrackContainer<PCObject> > _tracks);

private:
    inline float SIGN(float x) {return (x >= 0.0f) ? +1.0f : -1.0f;}
    inline float NORM(float a, float b, float c, float d) {return sqrt(a * a + b * b + c * c + d * d);}

public:
    shared_ptr< TrackContainer<PCObject> > tracks;

    void toPointCloudXYZI(Cloud &cloudOut);
    visualization_msgs::MarkerArray toMarkerGMMs();
    visualization_msgs::Marker toMarkerEdges();
};

#endif // PCPUBLISHER_H
