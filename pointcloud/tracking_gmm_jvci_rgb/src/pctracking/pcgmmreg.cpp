#include "pcgmmreg.h"
#include <vcl_iostream.h>
#include <vnl/vnl_vector.h>
#include <vnl/vnl_matrix.h>
#include <vnl/algo/vnl_lbfgsb.h>
#include "pcgmmreg_func.h"

PCGMMReg::PCGMMReg()
{   
    func = new PCGMMReg_func;
}

void PCGMMReg::init(PCObject* _model, PCObject* _scene, int _maxEval, FuncPub _funcPub)
{
    model = _model;
    scene = _scene;
    funcPub = _funcPub;
    maxEval = _maxEval;
    m = model->gmm.size();
    s = scene->gmm.size();
    transformed_model = *_model;
//    transformed_model.scale = model->scale;
//    transformed_model.percent = model->percent;
    n = m;
    d = 3;

    // translation matrix
    param_norm.set_size(3);
    param_norm.fill(0.);
    for(int i=0;i<model->gmm.size();i++){
        param_norm[0] += model->gmm.at(i).mean[0];
        param_norm[1] += model->gmm.at(i).mean[1];
        param_norm[2] += model->gmm.at(i).mean[2];
    }
    param_norm[0] /= model->gmm.size();
    param_norm[1] /= model->gmm.size();
    param_norm[2] /= model->gmm.size();

    // translate model and scene
    translate(model, -param_norm);
    translate(scene, -param_norm);

    param_rigid.set_size(7);
    param_rigid.fill(0);
    //    param_rigid[3] = 1;

    param_rigid[0] = 0.;
    param_rigid[1] = 0.;
    param_rigid[2] = 0.;
    param_rigid[3] = 1.;
    param_rigid[4] = 0.;
    param_rigid[5] = 0.;
    param_rigid[6] = 0.;
    perform_transform(param_rigid);
}

void PCGMMReg::translate(PCObject* object, vnl_vector<double> param)
{
    for(int i=0;i<object->gmm.size();i++){
        object->gmm.at(i).mean[0] += param[0];
        object->gmm.at(i).mean[1] += param[1];
        object->gmm.at(i).mean[2] += param[2];
    }
}

void PCGMMReg::run()
{
    // start
    start_registration(param_rigid);
    //    start_registration2(param_rigid);
    perform_transform(param_rigid);
    translate(&transformed_model, param_norm);
    translate(scene, param_norm);

}

void PCGMMReg::set_bound() {
    if (d == 2) {
        nbd.set_size(3);
        nbd[0] = 0;
        nbd[1] = 0;
        nbd[2] = 2;
        lower_bound.set_size(3);
        lower_bound.fill(0);
        lower_bound[2] = -3.1415926;
        upper_bound.set_size(3);
        upper_bound.fill(0);
        upper_bound[2] = 3.1415926;
    } else if (d == 3) {
        nbd.set_size(7);
        nbd.fill(0);
        for (int i = 0; i < 7; ++i) {
            nbd[i] = 2;
            //            nbd[i] = 0;
        }
        nbd[3] = 0;
        lower_bound.set_size(7);
        lower_bound.fill(0);
        for (int i = 0; i < 4; ++i) {
            lower_bound[i] = -0.1;
        }
        upper_bound.set_size(7);
        upper_bound.fill(0);
        for (int i = 0; i < 4; ++i) {
            upper_bound[i] = 0.1;
        }
        for (int i=4;i<7;i++){
            lower_bound[i] = -0.1;
            upper_bound[i] = 0.1;
        }
    }
}

void PCGMMReg::start_registration2(vnl_vector<double>& params)
{
    func->set_gmmreg(this);
    vnl_vector<double> g;
    g.set_size(6);
    g.fill(0);
    for(int i=0;i<100;i++){
        //        vcl_cout << "1 " << vcl_endl;


        double fv = func->f(params);
        //        vcl_cout << "2 " << vcl_endl;
        func->gradf(params, g);
        //        vcl_cout << "3 " << vcl_endl;
        params[0] = params[0] - g[0];
        params[1] = params[1] - g[1];
        params[2] = params[2] - g[2];
        params[3] = params[3] - g[3];
        params[4] = params[4] - g[4];
        params[5] = params[5] - g[5];

        vcl_cout <<"cnt: "<<i<< " f: " << fv << " g: " << g<< vcl_endl;
        vcl_cout << "Solution: " << params << vcl_endl;
    }
}

void PCGMMReg::start_registration(vnl_vector<double>& params)
{
    vnl_lbfgsb minimizer(*func);
    set_bound();
    minimizer.set_bound_selection(nbd);
    minimizer.set_lower_bound(lower_bound);
    minimizer.set_upper_bound(upper_bound);

    //: Set the projected gradient tolerance.
    // When the projected gradient vector has no component larger than
    // the given value convergence is assumed.  The default value is
    // 1e-5.
//    minimizer.set_projected_gradient_tolerance(1e-2);

    //: Set the cost function convergence factor.
    // When an iteration changes the function value by an amount smaller than
    // this factor times the machine epsilon (scaled by function magnitude)
    // convergence is assumed.  The default value is 1e+7.
    minimizer.set_cost_function_convergence_factor(1e+13);

    //: Set the convergence tolerance on F (sum of squared residuals).
    // When the differences in successive RMS errors is less than this, the
    // routine terminates.  So this is effectively the desired precision of your
    // minimization.  Setting it too low wastes time, too high might cause early
    // convergence.  The default of 1e-9 is on the safe side, but if speed is an
    // issue, you can try raising it.
//    minimizer.set_f_tolerance(1e+11);

    //: Set the convergence tolerance on X.
    //  When the length of the steps taken in X are about this long, the routine
    // terminates.  The default is 1e-8, which should work for many problems,
    // but if you can get away with 1e-4, say, minimizations will be much quicker.
//    minimizer.set_x_tolerance(1e-4);

    //: Set the convergence tolerance on Grad(F)' * F.
//    minimizer.set_g_tolerance();

    //: Set the step length for FD Jacobian.
    // Be aware that set_x_tolerance will reset this to xtol * 0.001.
    // The default is 1e-11.
//    minimizer.set_epsilon_function(1e-11);

    //double	fxval;
    func->set_gmmreg(this);
    minimizer.set_max_function_evals(maxEval);

    minimizer.set_trace(1);
    //    std::cout<<"factor"<<minimizer.get_cost_function_convergence_factor()<<std::endl;

    //    minimizer.set_projected_gradient_tolerance(0.0000000000001);
    //    minimizer.set_f_tolerance(0.005);
    //        minimizer.set_cost_function_convergence_factor(10000000000000000000000000000000000000000000000);
    //    std::cout << "param : " << params << std::endl;
    // For more options, see
    // http://public.kitware.com/vxl/doc/release/core/vnl/html/vnl__nonlinear__minimizer_8h-source.html
    minimizer.minimize(params);

    /*
      fxval = func->f( params );
      vcl_cout << "Minimized to " << fxval << vcl_endl
              << "Iterations: " << minimizer.get_num_iterations() << "; "
              << "Evaluations: " << minimizer.get_num_evaluations() << vcl_endl;
      */

    vcl_cout << "Solution: " << params << vcl_endl;
}


void PCGMMReg::perform_transform(const vnl_vector<double> &x)
{
    assert((d==2)||(d==3));
    vnl_matrix<double> translation;
    vnl_matrix<double> rotation;
    vnl_matrix<double> ones;
    ones.set_size(m,1);
    ones.fill(1);
    if (d == 2) {
        rotation.set_size(2,2);
        double theta = x[2];
        rotation[0][0] = cos(theta);
        rotation[0][1] = -sin(theta);
        rotation[1][0] = sin(theta);
        rotation[1][1] = cos(theta);
        translation.set_size(1,2);
        translation[0][0] = x[0];
        translation[0][1] = x[1];
    } else if (d == 3) {
        rotation.set_size(3,3);
        vnl_vector<double> q;
        q.set_size(4);
        for (int i=0;i<4;++i) q[i] = x[i];
        Gaussian::quaternion2rotation(q, rotation);
        translation.set_size(1,3);    modelpoints.set_size(m,d);
        for(int i=0;i<m;i++){
            modelpoints.put(i,0, model->gmm.at(i).mean[0]);
            modelpoints.put(i,1, model->gmm.at(i).mean[1]);
            modelpoints.put(i,2, model->gmm.at(i).mean[2]);
        }
        translation[0][0] = x[4];
        translation[0][1] = x[5];
        translation[0][2] = x[6];
    }
    transformed_model.gmm.clear();
    vnl_matrix<double> output_model(m,3);
    for(int i=0;i<m;i++){
        Gaussian gauss;
        gauss = model->gmm.at(i);
        vnl_matrix<double> mean(1,3), transmean(1,3);
        mean.put(0,0, gauss.mean[0]);
        mean.put(0,1, gauss.mean[1]);
        mean.put(0,2, gauss.mean[2]);
        transmean = mean*rotation.transpose() + translation;
        gauss.mean[0] = transmean[0][0];
        gauss.mean[1] = transmean[0][1];
        gauss.mean[2] = transmean[0][2];

        vnl_matrix<double> covariance(3,3), transcovariance(3,3);
        for(int j=0;j<3;j++)
            for(int k=0;k<3;k++)
                covariance.put(j,k, gauss.covariance(j,k));
        transcovariance = rotation*covariance*rotation.transpose();
        for(int j=0;j<3;j++)
            for(int k=0;k<3;k++)
                gauss.covariance(j,k) = covariance[j][k];
        transformed_model.gmm.push_back(gauss);
        output_model.put(i,0,gauss.mean[0]+param_norm[0]);
        output_model.put(i,1,gauss.mean[1]+param_norm[1]);
        output_model.put(i,2,gauss.mean[2]+param_norm[2]);
    }
    model->setTransParam(x);
    param_rigid = x;

    funcPub(output_model);
}
