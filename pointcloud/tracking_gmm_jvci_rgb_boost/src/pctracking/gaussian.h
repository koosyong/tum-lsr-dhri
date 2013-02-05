#ifndef GAUSSIAN_H
#define GAUSSIAN_H

//#include "pcobject.h"

#include <pcl/common/time.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/octree/octree.h>

#include <vector>
#include <Eigen/Dense>
#include <vnl/vnl_vector.h>
#include <vnl/vnl_matrix.h>

using namespace std;


typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;


class Point
{
public:
    Point();
    Point(PointT point);

public:
    Eigen::Vector3d pos;
    Eigen::Vector3d rgb;
    int id;
};

class Gaussian
{
public:
    Gaussian();
    Gaussian(CloudPtr points, double scale);

public:
    Eigen::VectorXd mean;
    Eigen::Vector3d velocity;
    Eigen::Vector3d eigenvalues;
    Eigen::Matrix3d eigenvectors;
    Eigen::MatrixXd covariance;
    Eigen::MatrixXd cov_inverse;
    double cov_determinant;

    Eigen::VectorXd predictive_mean;
    Eigen::MatrixXd predictive_covariance;


    double weight;
    int nPoint;
    bool isEmpty;

    vnl_matrix<double> translation;
    vnl_matrix<double> rotation;

public:
    double evalPoint(Point point);
    void updateParam(vnl_vector<double> newParam);
    void initPrediction();



    static void quaternion2rotation(vnl_vector<double> q, vnl_matrix<double>& R, vnl_matrix<double>& g1, vnl_matrix<double>& g2, vnl_matrix<double>& g3, vnl_matrix<double>& g4);
    static void quaternion2rotation(vnl_vector<double> q, vnl_matrix<double>& R);

};

#endif // GAUSSIAN_H
