#include "gaussian.h"

Point::Point()
{

}

Point::Point(PointT point, int _dim)
{
    dim = _dim;
//    if(dim == 3){
//        pos[0] = (double)point.x;
//        pos[1] = (double)point.y;
//        pos[2] = (double)point.z;
//    }
//    if(dim == 6){
        pos[0] = (double)point.x;
        pos[1] = (double)point.y;
        pos[2] = (double)point.z;
        rgb[0] = (double)point.r / 3000;
        rgb[1] = (double)point.g / 3000;
        rgb[2] = (double)point.b / 3000;
//    }
    //    cout<<"r "<<(double)point.r<<" g "<<(double)point.g<<" b "<<(double)point.b<<endl;
}

Gaussian::Gaussian()
{

}

Gaussian::Gaussian(int _dim)
{
    dim = _dim;
    isEmpty = 0;
    rotation.set_size(3,3);
    rotation[0][0] = 1;
    rotation[0][1] = 0;
    rotation[0][2] = 0;
    rotation[1][0] = 0;
    rotation[1][1] = 1;
    rotation[1][2] = 0;
    rotation[2][0] = 0;
    rotation[2][1] = 0;
    rotation[2][2] = 1;

    translation.set_size(1,3);
    translation[0][0] = 0;
    translation[0][1] = 0;
    translation[0][2] = 0;

    if(dim == 6){
        mean = Eigen::VectorXd(6);
        covariance = Eigen::MatrixXd(6,6);
        cov_inverse = Eigen::MatrixXd(6,6);
    }
    if(dim == 3){
        mean = Eigen::VectorXd(3);
        covariance = Eigen::MatrixXd(3,3);
        cov_inverse = Eigen::MatrixXd(3,3);
    }
}

void Gaussian::initPrediction()
{
    predictive_mean = mean;
    predictive_covariance = covariance;
}
/*
Gaussian::Gaussian(CloudPtr points, double scale)
{
    mean = Eigen::VectorXd(6);
    covariance = Eigen::Matrix3d(6,6);
    cov_inverse = Eigen::Matrix3d(6,6);

    isEmpty = 0;
    rotation.set_size(3,3);
    rotation[0][0] = 1;
    rotation[0][1] = 0;
    rotation[0][2] = 0;
    rotation[1][0] = 0;
    rotation[1][1] = 1;
    rotation[1][2] = 0;
    rotation[2][0] = 0;
    rotation[2][1] = 0;
    rotation[2][2] = 1;

    translation.set_size(1,3);
    translation[0][0] = 0;
    translation[0][1] = 0;
    translation[0][2] = 0;


    vector<Gaussian> set;
    nPoint = points->points.size();


    for(int i=0;i<nPoint;i++){
        Gaussian gaussianPoint;
        gaussianPoint.mean[0] = (double)points->points.at(i).x;
        gaussianPoint.mean[1] = (double)points->points.at(i).y;
        gaussianPoint.mean[2] = (double)points->points.at(i).z;

        gaussianPoint.mean[3] = (double)points->points.at(i).r / 2560;
        gaussianPoint.mean[4] = (double)points->points.at(i).g / 2560;
        gaussianPoint.mean[5] = (double)points->points.at(i).b / 2560;

        gaussianPoint.covariance(0,0) = scale*scale;
        gaussianPoint.covariance(0,1) = 0;
        gaussianPoint.covariance(0,2) = 0;
        gaussianPoint.covariance(0,3) = 0;
        gaussianPoint.covariance(0,4) = 0;
        gaussianPoint.covariance(0,5) = 0;

        gaussianPoint.covariance(1,0) = 0;
        gaussianPoint.covariance(1,1) = scale*scale;
        gaussianPoint.covariance(1,2) = 0;
        gaussianPoint.covariance(1,3) = 0;
        gaussianPoint.covariance(1,4) = 0;
        gaussianPoint.covariance(1,5) = 0;

        gaussianPoint.covariance(2,0) = 0;
        gaussianPoint.covariance(2,1) = 0;
        gaussianPoint.covariance(2,2) = scale*scale;
        gaussianPoint.covariance(2,3) = 0;
        gaussianPoint.covariance(2,4) = 0;
        gaussianPoint.covariance(2,5) = 0;

        gaussianPoint.covariance(3,0) = 0;
        gaussianPoint.covariance(3,1) = 0;
        gaussianPoint.covariance(3,2) = 0;
        gaussianPoint.covariance(3,3) = scale*scale;
        gaussianPoint.covariance(3,4) = 0;
        gaussianPoint.covariance(3,5) = 0;

        gaussianPoint.covariance(4,0) = 0;
        gaussianPoint.covariance(4,1) = 0;
        gaussianPoint.covariance(4,2) = 0;
        gaussianPoint.covariance(4,3) = 0;
        gaussianPoint.covariance(4,4) = scale*scale;
        gaussianPoint.covariance(4,5) = 0;

        gaussianPoint.covariance(5,0) = 0;
        gaussianPoint.covariance(5,1) = 0;
        gaussianPoint.covariance(5,2) = 0;
        gaussianPoint.covariance(5,3) = 0;
        gaussianPoint.covariance(5,4) = 0;
        gaussianPoint.covariance(5,5) = scale*scale;

        gaussianPoint.weight = 1./(double)nPoint;
        gaussianPoint.nPoint = 1;
        //        gaussianPoint.cov_determinant = gaussianPoint.covariance.determinant();
        gaussianPoint.cov_inverse = gaussianPoint.covariance.inverse();
        set.push_back(gaussianPoint);
    }
    Eigen::Vector3d t;
    t[0] = t[1] = t[2] = 0.;
    for(int i = 0;i<nPoint;i++){
        t = t + set.at(i).mean * set.at(i).weight;
    }
    mean = t;

    Eigen::Matrix3d cov_set = Eigen::Matrix3d::Zero();
    for(int i = 0;i<nPoint;i++){
        cov_set = cov_set + set.at(i).weight * (set.at(i).covariance + (mean-set.at(i).mean)*(mean-set.at(i).mean).transpose());
    }
    covariance = cov_set;

    cov_determinant = covariance.determinant();
    cov_inverse = covariance.inverse();
    weight = 1.0;

    initPrediction();
}
*/

void Gaussian::quaternion2rotation(vnl_vector<double> q, vnl_matrix<double>& R, vnl_matrix<double>& g1, vnl_matrix<double>& g2, vnl_matrix<double>& g3, vnl_matrix<double>& g4){
    double x,y,z,r;
    double x2,y2,z2,r2;
    x = q[0];  y = q[1];  z=q[2];  r = q[3];
    x2 = q[0] * q[0];
    y2 = q[1] * q[1];
    z2 = q[2] * q[2];
    r2 = q[3] * q[3];
    // fill diagonal terms
    R(0,0) = r2 + x2 - y2 - z2;
    R(1,1) = r2 - x2 + y2 - z2;
    R(2,2) = r2 - x2 - y2 + z2;
    // fill off diagonal terms
    R(0,1) = 2 * (x*y + r*z);
    R(0,2) = 2 * (z*x - r*y);
    R(1,2) = 2 * (y*z + r*x);
    R(1,0) = 2 * (x*y - r*z);
    R(2,0) = 2 * (z*x + r*y);
    R(2,1) = 2 * (y*z - r*x);
    double ss = (x2+y2+z2+r2);
    R = R/ss;
    double ssss = ss*ss;

    // derivative of R(0,0) = r2 + x2 - y2 - z2;
    g1(0,0) = 4*x*(y2+z2)/ssss;  g2(0,0) = -4*y*(x2+r2)/ssss;
    g3(0,0) = -4*z*(x2+r2)/ssss;  g4(0,0) = 4*r*(y2+z2)/ssss;
    // derivative of R(1,1) = r2 - x2 + y2 - z2;
    g1(1,1) = -4*x*(y2+r2)/ssss;  g2(1,1) = 4*y*(x2+z2)/ssss;
    g3(1,1) = -4*z*(y2+r2)/ssss;  g4(1,1) = 4*r*(x2+z2)/ssss;
    // derivative of R(2,2) = r2 - x2 - y2 + z2;
    g1(2,2) = -4*x*(z2+r2)/ssss;  g2(2,2) = -4*y*(r2+z2)/ssss;
    g3(2,2) = 4*z*(x2+y2)/ssss;  g4(2,2) = 4*r*(x2+y2)/ssss;

    // fill off diagonal terms
    // derivative of R(0,1) = 2 * (xy + rztrasformedModel);
    g1(0,1) = 2*y/ss - 2*x*R(0,1)/ssss;
    g2(0,1) = 2*x/ss - 2*y*R(0,1)/ssss;
    g3(0,1) = 2*r/ss - 2*z*R(0,1)/ssss;
    g4(0,1) = 2*z/ss - 2*r*R(0,1)/ssss;
    // derivative of R(0,2) = 2 * (zx - ry);
    g1(0,2) = 2*z/ss - 2*x*R(0,2)/ssss;
    g2(0,2) = -2*r/ss - 2*y*R(0,2)/ssss;
    g3(0,2) = 2*x/ss - 2*z*R(0,2)/ssss;
    g4(0,2) = -2*y/ss - 2*r*R(0,2)/ssss;
    // derivative of R(1,2) = 2 * (yz + rx);
    g1(1,2) = 2*r/ss - 2*x*R(1,2)/ssss;
    g2(1,2) = 2*z/ss - 2*y*R(1,2)/ssss;
    g3(1,2) = 2*y/ss - 2*z*R(1,2)/ssss;
    g4(1,2) = 2*x/ss - 2*r*R(1,2)/ssss;
    // derivative of R(1,0) = 2 * (xy - rz);
    g1(1,0) = 2*y/ss - 2*x*R(1,0)/ssss;
    g2(1,0) = 2*x/ss - 2*y*R(1,0)/ssss;
    g3(1,0) = -2*r/ss - 2*z*R(1,0)/ssss;
    g4(1,0) = -2*z/ss - 2*r*R(1,0)/ssss;
    // derivative of R(2,0) = 2 * (zx + ry);
    g1(2,0) = 2*z/ss - 2*x*R(2,0)/ssss;
    g2(2,0) = 2*r/ss - 2*y*R(2,0)/ssss;
    g3(2,0) = 2*x/ss - 2*z*R(2,0)/ssss;
    g4(2,0) = 2*y/ss - 2*r*R(2,0)/ssss;
    // derivative of R(2,1) = 2 * (yz - rx);
    g1(2,1) = -2*r/ss - 2*x*R(2,1)/ssss;
    g2(2,1) = 2*z/ss - 2*y*R(2,1)/ssss;
    g3(2,1) = 2*y/ss - 2*z*R(2,1)/ssss;
    g4(2,1) = -2*x/ss - 2*r*R(2,1)/ssss;
}

void Gaussian::quaternion2rotation(vnl_vector<double> q, vnl_matrix<double>& R){
    double x,y,z,r;
    double x2,y2,z2,r2;
    x = q[0];  y = q[1];  z=q[2];  r = q[3];
    x2 = q[0] * q[0];
    y2 = q[1] * q[1];
    z2 = q[2] * q[2];
    r2 = q[3] * q[3];
    // fill diagonal terms
    R(0,0) = r2 + x2 - y2 - z2;
    R(1,1) = r2 - x2 + y2 - z2;
    R(2,2) = r2 - x2 - y2 + z2;
    // fill off diagonal terms
    R(0,1) = 2 * (x*y + r*z);
    R(0,2) = 2 * (z*x - r*y);
    R(1,2) = 2 * (y*z + r*x);
    R(1,0) = 2 * (x*y - r*z);
    R(2,0) = 2 * (z*x + r*y);
    R(2,1) = 2 * (y*z - r*x);
    double ss = (x2+y2+z2+r2);
    R = R/ss;
}
/*
double Gaussian::evalPoint(Point point)
{
    double det = cov_determinant;
    Eigen::Matrix3d inv = cov_inverse;

    double a = (point.pos-mean).transpose()*inv*(point.pos-mean);
    return exp(-0.5*a);

    //    return exp(-0.5*a) / sqrt(pow(2*pi,6));

}
*/
void Gaussian::updateParam(vnl_vector<double> newParam)
{
    // transformation quaternion to matrix and update roation, translation parameters
    vnl_vector<double> q;
    q.set_size(4);
    for (int i=0;i<4;++i) q[i] = newParam[i];
    vnl_matrix<double> rot;
    rot.set_size(3,3);
    Gaussian::quaternion2rotation(q, rot);
    vnl_matrix<double> trans;
    trans.set_size(1,3);
    trans[0][0] = newParam[4];
    trans[0][1] = newParam[5];
    trans[0][2] = newParam[6];

    // update param
    translation += trans;
    rotation = rot * rotation;

    // prediction
    vnl_matrix<double> m(1,3), pred_mean(1,3);
    m.put(0,0, mean[0]);
    m.put(0,1, mean[1]);
    m.put(0,2, mean[2]);
    pred_mean = m * rotation.transpose() + translation;
    predictive_mean[0] = pred_mean[0][0];
    predictive_mean[1] = pred_mean[0][1];
    predictive_mean[2] = pred_mean[0][2];



    vnl_matrix<double> cov(3,3), pred_cov;
    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            cov.put(i,j, covariance.col(i)[j]);

    pred_cov = rotation * cov;
    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            predictive_covariance(i,j) = pred_cov.get(i,j);


}
