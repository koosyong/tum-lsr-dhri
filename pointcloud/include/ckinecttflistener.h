#ifndef CKINECTTFLISTENER_H
#define CKINECTTFLISTENER_H

#include "ros/ros.h"
#include "tf/transform_listener.h"
#include <string>
using namespace std;

class CKinectTFListener
{
public:
    CKinectTFListener();

public:
    void listen();

private:
    tf::StampedTransform transform(string target, string source);

private:
    tf::TransformListener *listener;

};

#endif // CKINECTTFLISTENER_H
