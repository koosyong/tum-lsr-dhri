#ifndef PCOBJECTCONTAINER_H
#define PCOBJECTCONTAINER_H

// pcl
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include <pcl_ros/transforms.h>

#include "pcobject.h"
#include <boost/shared_ptr.hpp>
using namespace boost;

class PCObjectContainer
{
public:
    PCObjectContainer();
    PCObjectContainer(CloudPtr _pCloud);    
    ~PCObjectContainer();

public:
    inline int numObjects(){return objects.size();};
    bool deleteObject(int id);
    void makeNewObject(PCObject& object);
    void initGMM(double scale, double percent);

private:
    void makingObjects();

public:
    vector< shared_ptr<PCObject> > objects;
    int num;

private:
    CloudPtr pCloud;
    double scale;


};

#endif // PCOBJECTCONTAINER_H
