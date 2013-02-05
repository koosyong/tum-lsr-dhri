#ifndef TRACKINGGMM_H
#define TRACKINGGMM_H

#include "ckinectpcreceiver.h"

typedef pcl::PointXYZRGB PointT;
typedef CKinectPCReceiver<PointT> PCReceiver;

class TrackingGMM
{
public:
    TrackingGMM();
};

#endif // TRACKINGGMM_H
