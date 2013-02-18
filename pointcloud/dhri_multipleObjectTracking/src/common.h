#ifndef COMMON_H
#define COMMON_H

// pcl
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include <pcl_ros/transforms.h>

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;

#endif // COMMON_H
