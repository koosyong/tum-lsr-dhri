/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Telerobotics and Control Laboratory(TCL), KAIST, Korea (http://robot.kaist.ac.kr)
 *                      Cluster of Excellence Cognition for Technical Systems(CoTeSys), TUM, Germany (http://cotesys.org)
 *  Author : Koosy (http://koosy.blogspot.com, koosyong@gmail.com)
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef CKINECT_H
#define CKINECT_H

// ros
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// pcl
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>

// general
#include <iostream>
using namespace std;

// Useful macros
#define FPS_CALC(_WHAT_) \
    do \
{ \
    static unsigned count = 0;\
    static double last = pcl::getTime ();\
    double now = pcl::getTime (); \
    ++count; \
    if (now - last >= 1.0) \
{ \
    ROS_INFO("FPS of %s: %f", _WHAT_, double(count)/double(now - last)); \
    count = 0; \
    last = now; \
    } \
    }while(false)

template<typename PointT>
class CKinect
{
public:
    typedef pcl::PointCloud<PointT> Cloud;
    typedef typename Cloud::Ptr CloudPtr;
    typedef typename Cloud::ConstPtr CloudConstPtr;

public:
    CKinect(string _paramType_subTopic, string _paramType_pubTopic, string _paramValue_subTopic, string _paramValue_pubTopic);
    void run();

protected:
    virtual void init();
    virtual void processing()=0;

public:
    static void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input);
    static CKinect<PointT>* parent;    

protected:
    ros::NodeHandle nh;
    bool isParam;
    string paramType_subTopic;
    string paramType_pubTopic;
    string paramValue_subTopic;
    string paramValue_pubTopic;
    CloudPtr pCloud_input;
    CloudPtr pCloud_output;
    bool isPub;

private:
    ros::Subscriber sub;
    ros::Publisher pub;

};

#include "ckinect.hpp"
#endif // CKINECT_H
