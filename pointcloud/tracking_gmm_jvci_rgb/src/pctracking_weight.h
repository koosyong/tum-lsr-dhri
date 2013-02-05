#ifndef PCTRACKING_WEIGHT_H
#define PCTRACKING_WEIGHT_H

#include "pcobject.h"

#define SQR(X)  ((X)*(X))
#define pi 3.141592

typedef struct{
    Eigen::Matrix3d covariance;
    Eigen::Vector3d eigenvalues;
    Eigen::Matrix3d eigenvectors;
    Eigen::Vector3d mean;
} GAUSS1;


GAUSS1 gauss1(PCObject &o);
double weight_cov(PCObject &o1, PCObject &o2);
double weight_l2_gmm1(PCObject &o1, PCObject &o2);
double weight_l2(PCObject &o1, PCObject &o2);
double weight_l2_points(PCObject &o1, PCObject &o2);
double weight_energy2(PCObject &o1, PCObject &o2);
double weight_gaussian(Gaussian &g1, Gaussian &g2);
double weight_gaussian_predictive(Gaussian &g1, Gaussian &g2);

#endif // PCTRACKING_WEIGHT_H
