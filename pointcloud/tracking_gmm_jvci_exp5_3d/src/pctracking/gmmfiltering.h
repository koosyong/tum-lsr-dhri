#ifndef GMMFILTERING_H
#define GMMFILTERING_H

#include "pcobjectcontainer.h"

class GMMFiltering
{
public:
    GMMFiltering();
    GMMFiltering(PCObject prior, PCObject measurement, vnl_vector<double> param);
    GMMFiltering(PCObject predictive, PCObject measurement);
    GMMFiltering(PCObject predictive, CloudPtr measurement, double minEvalProb, CloudPtr& unmatchedPoints, double scale); // locally weighted gaussian mixture filter without gmm simplification

private:
    PCObject perform_transform(PCObject prior, const vnl_vector<double> &x);

public:
    PCObject predictive, posterior;

private:
    double scale_noise, scale_measurement;
    Eigen::Matrix3d cov_noise;
};

#endif // GMMFILTERING_H
