/*=========================================================================
$Author: bingjian $
$Date: 2011-09-06 07:51:56 +0200 (Tue, 06 Sep 2011) $
$Revision: 141 $
=========================================================================*/

/**
 * \file gmmreg_rigid_func.cpp
 * \brief  The definition of the class gmmreg_rigid_func
 */

#include "gmmreg_utils.h"
#include "gmmreg_rigid_func.h"
#include <iostream>
#define SQR(X)  ((X)*(X))
#define pi 3.141592
double gmmreg_rigid_func::eval(double& f, vnl_matrix<double>& g) {
    /* L2 version and KC version are equivalent in the rigid case */
    g =  -g;
    return -f;
}


double gmmreg_rigid_func::f(const vnl_vector<double>& x) {
  // std::cout << "f begin" << std::endl;
  gmmreg->perform_transform(x);
  // std::cout << "transform done" << std::endl;
  double energy = GaussTransform(gmmreg->transformed_model,
      gmmreg->scene, scale, gradient);
  // std::cout << "gauss transform done" << std::endl;
  return eval(energy, gradient);
  // std::cout << "f end" << std::endl;
}
/*
double gmmreg_rigid_func::f(const vnl_vector<double>& x) {
    // std::cout << "f begin" << std::endl;
    gmmreg->perform_transform(x);
    // std::cout << "transform done" << std::endl;
    double energy1 = GaussTransform(gmmreg->transformed_model,
                                    gmmreg->transformed_model, scale, gradient1);
    double energy2 = GaussTransform(gmmreg->transformed_model,
                                    gmmreg->scene, scale, gradient2);
    double energy3 = GaussTransform(gmmreg->scene,
                                    gmmreg->scene, scale);
    //    gradient = -gradient;
    //  eval(energy2, gradient);

    int mm = gmmreg->transformed_model.rows();
    int nn = gmmreg->scene.rows();
    int ddim = 3;
    double *A = gmmreg->transformed_model.data_block();
    double *B = gmmreg->scene.data_block();

    double sscale = SQR(scale);
    double sum_x = 0.;
    double sum_y = 0.;
    double sum_z = 0.;
    for(int i=0;i<mm;i++){
        for(int j=0;j<nn;j++){
            for(int d=0;d<ddim;d++){
                double uf2 = SQR(A[i*ddim+d]);
                double ug2 = SQR(B[j*ddim+d]);
                double ufg2 = SQR(A[i * ddim + d] + B[j * ddim + d]);
                double a = (uf2 + ug2)/2 - ufg2/4;
                if(d == 0) sum_x += exp(-1.0 * a / (2*sscale)) * (-1.0*a+sscale);
                if(d == 1) sum_y += exp(-1.0 * a / (2*sscale)) * (-1.0*a+sscale);
                if(d == 2) sum_z += exp(-1.0 * a / (2*sscale)) * (-1.0*a+sscale);
            }
        }
    }
    double cost_grad_x = 1/(8*scale*pi*sqrt(pi)) - (2/mm*nn)*(1/sscale*scale*4*pi*sqrt(2*pi))*sum_x;
    double cost_grad_y = 1/(8*scale*pi*sqrt(pi)) - (2/mm*nn)*(1/sscale*scale*4*pi*sqrt(2*pi))*sum_y;
    double cost_grad_z = 1/(8*scale*pi*sqrt(pi)) - (2/mm*nn)*(1/sscale*scale*4*pi*sqrt(2*pi))*sum_z;
    double cost_grad = cost_grad_x + cost_grad_y + cost_grad_z;

    //    cost_grad = gradient.get_column(0).sum() + gradient.get_column(1).sum() + gradient.get_column(2).sum();
    double alpha = 1; // 0.1 good
    double cost = ((energy1 - 2*energy2 + energy3)*alpha + cost_grad*(1-alpha));
    //    double cost = 0-energy2*alpha + cost_grad*(1-alpha);

    // ggradient
    vnl_matrix<double> ggradient(mm,ddim);
    ggradient.fill(0);
    double *ggrad = ggradient.data_block();

    for (int i = 0; i < mm * ddim; ++i) {
        ggrad[i] = 0;
    }

    for (int i = 0; i < mm; ++i) {
        sum_x = 0.;
        sum_y = 0.;
        sum_z = 0.;
        for (int j = 0; j < nn; ++j) {
            for (int d = 0; d < ddim; ++d) {
                double uf2 = SQR(A[i*ddim+d]);
                double ug2 = SQR(B[j*ddim+d]);
                double ufg2 = SQR(A[i * ddim + d] + B[j * ddim + d]);
                double a = (uf2 + ug2)/2 - ufg2/4;
                if(d == 0) sum_x += exp(-1.0 * a / (2*sscale)) * (-1.0*B[j * ddim + d]) * (B[j * ddim + d] - (sscale-a)/(2*sscale));
                if(d == 1) sum_y += exp(-1.0 * a / (2*sscale)) * (-1.0*B[j * ddim + d]) * (B[j * ddim + d] - (sscale-a)/(2*sscale));
                if(d == 2) sum_z += exp(-1.0 * a / (2*sscale)) * (-1.0*B[j * ddim + d]) * (B[j * ddim + d] - (sscale-a)/(2*sscale));
            }
        }
        ggrad[i*ddim+0] = sum_x * (-2.0) / (mm*nn*scale*sscale*4*pi*sqrt(2*pi));
        ggrad[i*ddim+1] = sum_y * (-2.0) / (mm*nn*scale*sscale*4*pi*sqrt(2*pi));
        ggrad[i*ddim+2] = sum_z * (-2.0) / (mm*nn*scale*sscale*4*pi*sqrt(2*pi));
    }
//        std::cout << "gradient1 : " << gradient1 << std::endl;
    //    std::cout << "ggradient : " << ggradient << std::endl;


    // gradient
    gradient = (gradient1*2-gradient2*2)*alpha;
    ggradient = ggradient*(1-alpha);
    gradient = (gradient+ggradient);

//    std::cout << "cost: " << cost << std::endl;
//    std::cout << "gradient : " << gradient << std::endl;
    return cost;
    // std::cout << "f end" << std::endl;
}
*/
void gmmreg_rigid_func::gradf(const vnl_vector<double>& x,
                              vnl_vector<double>& g) {
    /*     case 'rigid2d'
 *         [f, grad] = rigid_costfunc(transformed_model, scene, scale);
 *         grad = grad';
 *         g(1) = sum(grad(1,:));
 *         g(2) = sum(grad(2,:));
 *         grad = grad*model;
 *         theta = param(3);
 *         r = [-sin(theta) -cos(theta);
 *              cos(theta)  -sin(theta)];
 *         g(3) = sum(sum(grad.*r));
 *
 */
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
        gm = gradient.transpose()*gmmreg->model;
        g[2] = 0;
        for (i=0;i<2;++i) {
            for (int j=0;j<2;++j) {
                g[2] += r[i][j]*gm[i][j];
            }
        }
    } else if (d == 3) { //rigid3d
        /*
*     case 'rigid3d'
*        [f,grad] = rigid_costfunc(transformed_model, scene, scale);
*         [r,gq] = quaternion2rotation(param(1:4));
*         grad = grad';
*         gm = grad*model;
*         g(1) = sum(sum(gm.*gq{1}));
*         g(2) = sum(sum(gm.*gq{2}));
*         g(3) = sum(sum(gm.*gq{3}));
*         g(4) = sum(sum(gm.*gq{4}));
*         g(5) = sum(grad(1,:));
*         g(6) = sum(grad(2,:));
*         g(7) = sum(grad(3,:));
*/
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
        quaternion2rotation(q,r,g1,g2,g3,g4);


        gm = gradient.transpose()*gmmreg->model;
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
//    std::cout<<"param:"<<g<<std::endl;
}
