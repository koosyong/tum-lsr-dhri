#ifndef PCTHEADER_H
#define PCTHEADER_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointCloud<pcl::PointXYZ> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;
typedef pcl::PointCloud<pcl::PointWithRange> CloudRange;
typedef std::vector<CloudPtr> VecCloud;

typedef struct{
    Eigen::Matrix3d covariance;
    Eigen::Vector3d eigenvalues;
    Eigen::Matrix3d eigenvectors;
    Eigen::Vector3d mean;
    int id;
} STATISTIC;

typedef struct : public STATISTIC{
//    Cloud cloud;
} OBJECT;


typedef struct {
    double left, right, top, bottom;
    double margin;
} TYPE_WS;

#endif // PCTHEADER_H
