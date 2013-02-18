#include "pcgmmreg_func.h"
//#include "gmmreg_utils.h"

#include <iostream>
#define SQR(X)  ((X)*(X))
#define pi 3.141592


double PCGMMReg_func::weight_l2(PCObject &model, PCObject &scene)
{
    // reference :
    // Robust Point Set Registration Using Gaussian Mixture Models
    // Bing Jina, and Baba C. Vemuri
    // IEEE Transactions on Pattern Analysis and Machine Intelligence 2010

    double energy1 = 0.;
    for(int i=0;i<m;i++){
        for(int j=0;j<m;j++){
            int dim = model.gmm.at(i).dim;
            Eigen::MatrixXd cov = model.gmm.at(i).covariance + model.gmm.at(j).covariance;
            Eigen::VectorXd mean = model.gmm.at(i).mean - model.gmm.at(j).mean;
            Eigen::MatrixXd invij = cov.inverse();
            double a = mean.transpose()*invij*mean;
            double gauss = 1./sqrt(pow(2*pi,dim)*cov.determinant())*exp(-0.5*a);
            energy1 += model.gmm.at(i).weight*model.gmm.at(j).weight*gauss;
        }
    }
//    cout<<"m "<<m<<endl;
//    cout<<"s "<<s<<endl;
    double energy2 = 0.;
    for(int i=0;i<m;i++){
        double sum[3] = {0.,0.,0.};
        for(int j=0;j<s;j++){
            int dim = model.gmm.at(i).dim;
            Eigen::MatrixXd cov = model.gmm.at(i).covariance + scene.gmm.at(j).covariance;
            Eigen::VectorXd mean = model.gmm.at(i).mean - scene.gmm.at(j).mean;
            Eigen::MatrixXd invij = cov.inverse();
            double a = mean.transpose()*invij*mean;
            double gauss = 1./sqrt(pow(2*pi,dim)*cov.determinant())*exp(-0.5*a);
            energy2 += model.gmm.at(i).weight*scene.gmm.at(j).weight*gauss;
//            cout<<"weight i "<<model.gmm.at(i).weight<<endl;
//            cout<<"weight j "<<scene.gmm.at(j).weight<<endl;
//            cout<<"a "<<a<<endl;
//            cout<<"gauss "<<gauss<<endl;


            // gradient [m,d]
            double derv_x = -0.5*(2*mean[0]*invij(0,0) + mean[1]*(invij(0,1)+invij(1,0)) + mean[2]*(invij(0,2)+invij(2,0)));
            double derv_y = -0.5*(mean[0]*(invij(1,0)+invij(0,1)) + 2*mean[1]*invij(1,1) + mean[2]*(invij(1,2)+invij(2,1)));
            double derv_z = -0.5*(mean[0]*(invij(2,0)+invij(0,2)) + mean[1]*(invij(2,1)+invij(1,2)) + 2*mean[2]*invij(2,2));

            sum[0] += scene.gmm.at(j).weight*gauss*derv_x;
            sum[1] += scene.gmm.at(j).weight*gauss*derv_y;
            sum[2] += scene.gmm.at(j).weight*gauss*derv_z;

        }
        gradient[i][0] = -2.*model.gmm.at(i).weight*sum[0];
        gradient[i][1] = -2.*model.gmm.at(i).weight*sum[1];
        gradient[i][2] = -2.*model.gmm.at(i).weight*sum[2];
    }
    double energy3 = 0.;
    for(int i=0;i<s;i++){
        for(int j=0;j<s;j++){
            int dim = scene.gmm.at(i).dim;
            Eigen::MatrixXd cov = scene.gmm.at(i).covariance + scene.gmm.at(j).covariance;
            Eigen::VectorXd mean = scene.gmm.at(i).mean - scene.gmm.at(j).mean;
            Eigen::MatrixXd invij = cov.inverse();
            double a = mean.transpose()*invij*mean;
            double gauss = 1./sqrt(pow(2*pi,dim)*cov.determinant())*exp(-0.5*a);
            energy3 += scene.gmm.at(i).weight*scene.gmm.at(j).weight*gauss;
        }
    }
    return energy1 - 2*energy2 + energy3;
//    cout<<"energy2 "<<energy2<<endl;
//    return -2*energy2;
}

// return l2 distance of gmms from the parameter x
double PCGMMReg_func::f(const vnl_vector<double>& x)
{
//    cout<<"param "<<x<<endl;
    gmmreg->perform_transform(x);   // get transformed_model
    // l2 distance between transformed_model and scene
    double l2 = weight_l2(gmmreg->transformed_model, *gmmreg->scene);
//    cout<<"l2 weight "<<l2<<endl;
    return l2;
    // gradient




//    double energy = GaussTransform(gmmreg->transformed_model,
//                                   gmmreg->scene, scale, gradient);
//    return eval(energy, gradient);

}

void PCGMMReg_func::gradf(const vnl_vector<double>& x,
                          vnl_vector<double>& g) {

    int i = 0;
    vnl_matrix<double> r;
    vnl_matrix<double> gm;
    if (d == 2) { //rigid2d
        g[0] = gradient.get_column(0).sum();
        g[1] = gradient.get_column(1).sum();
        r.set_size(2,2);
        double theta = x[2];
        r[0][0] = -sin(theta);
        r[0][1] = -cos(theta);
        r[1][0] = cos(theta);
        r[1][1] = -sin(theta);

        gm = gradient.transpose()*gmmreg->modelpoints;
        g[2] = 0;
        for (i=0;i<2;++i) {
            for (int j=0;j<2;++j) {
                g[2] += r[i][j]*gm[i][j];
            }
        }
    } else if (d == 3) { //rigid3d

        g[4] = gradient.get_column(0).sum();
        g[5] = gradient.get_column(1).sum();
        g[6] = gradient.get_column(2).sum();
        vnl_vector<double> q;
        q.set_size(4);
        for (i=0;i<4;++i) q[i] = x[i];
        r.set_size(3,3);
        vnl_matrix<double> g1,g2,g3,g4;
        g1.set_size(3,3);
        g2.set_size(3,3);
        g3.set_size(3,3);
        g4.set_size(3,3);
        Gaussian::quaternion2rotation(q,r,g1,g2,g3,g4);
        gm = gradient.transpose()*gmmreg->modelpoints;
        g[0] = 0;
        for (i=0;i<3;++i) {
            for (int j=0;j<3;++j) {
                g[0] += g1[i][j]*gm[i][j];
            }
        }
        g[1] = 0;
        for (i=0;i<3;++i) {
            for (int j=0;j<3;++j) {
                g[1] += g2[i][j]*gm[i][j];
            }
        }
        g[2] = 0;
        for (i=0;i<3;++i) {
            for (int j=0;j<3;++j) {
                g[2] += g3[i][j]*gm[i][j];
            }
        }
        g[3] = 0;
        for (i=0;i<3;++i) {
            for (int j=0;j<3;++j) {
                g[3] += g4[i][j]*gm[i][j];
            }
        }
    }
//    cout<<"g"<<g<<endl;
}
