#ifndef CKINECTTRANSFORM_H
#define CKINECTTRANSFORM_H

#include <ckinect.h>
#include <pcfilter.h>

typedef pcl::PointXYZRGB PointT;

class CKinectTransform : public CKinect<PointT>
{
public:
    CKinectTransform();

private:
    PCFilter<PointT> filter;

    // default param
    double paramValue_height;
    double paramValue_angle;

protected:
    virtual void processing();
    void init();
};

#endif // CKINECTTRANSFORM_H
