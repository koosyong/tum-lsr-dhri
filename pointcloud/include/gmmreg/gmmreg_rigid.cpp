/*=========================================================================
$Author: bingjian $
$Date: 2011-09-06 07:51:56 +0200 (Tue, 06 Sep 2011) $
$Revision: 141 $
=========================================================================*/

/**
 * \file gmmreg_rigid.cpp
 * \brief  The definition of the class gmmreg_rigid
 */

#include <assert.h>
#include <iostream>
#include <fstream>

#include <vcl_iostream.h>
//#include <vnl/vnl_random.h>
#include <vnl/vnl_vector.h>
#include <vnl/vnl_matrix.h>
#include <vnl/algo/vnl_lbfgsb.h>

#include "gmmreg_utils.h"
#include "gmmreg_rigid.h"

void gmmreg_rigid::rigidInit(int _level, char* _sigma, char* _max_function_evals)
{
    // set init param
    set_init_params();
    prepare_own_options(_level, _sigma, _max_function_evals);
    // prepare_basis_kernel
    prepare_basis_kernel();
}

void gmmreg_rigid::set_bound() {
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
    for (int i = 4; i < 7; ++i) {
      nbd[i] = 2;
    }
    lower_bound.set_size(7);
    lower_bound.fill(0);
    for (int i = 0; i < 4; ++i) {
      lower_bound[i] = -1;
    }    
    upper_bound.set_size(7);
    upper_bound.fill(0);
    for (int i = 0; i < 4; ++i) {
      upper_bound[i] = 1;
    }
    for (int i=4;i<7;i++){
        lower_bound[i] = -0.00001;
        upper_bound[i] = 0.00001;
    }
  }
}

void gmmreg_rigid::start_registration(vnl_vector<double>& params) {
  vnl_lbfgsb minimizer(*func);
  set_bound();
  minimizer.set_bound_selection(nbd);
  minimizer.set_lower_bound(lower_bound);
  minimizer.set_upper_bound(upper_bound);
  minimizer.set_projected_gradient_tolerance(0.0);
  //double	fxval;
  func->set_gmmreg(this);
  for (unsigned int k = 0; k < level; ++k) {
    func->set_scale(v_scale[k]);
    set_param(params);
    minimizer.set_max_function_evals(v_func_evals[k]);
//    minimizer.set_trace(1);
//    std::cout<<"factor"<<minimizer.get_cost_function_convergence_factor()<<std::endl;
    minimizer.set_projected_gradient_tolerance(0.001);
//    minimizer.set_f_tolerance(0.005);
//    minimizer.set_cost_function_convergence_factor(1000000);
//    std::cout << "param : " << params << std::endl;
    // For more options, see
    // http://public.kitware.com/vxl/doc/release/core/vnl/html/vnl__nonlinear__minimizer_8h-source.html
    minimizer.minimize(params);
    if (minimizer.get_failure_code() < 0) {
      /*
        fxval = func->f( params );
         vcl_cout << "break Minimized to " << fxval << vcl_endl
        << "Iterations: " << minimizer.get_num_iterations() << "; "
        << "Evaluations: " << minimizer.get_num_evaluations() << vcl_endl;
        vcl_cout << params << vcl_endl;
        */
      break;
    }
    /*
    fxval = func->f( params );
    vcl_cout << "Minimized to " << fxval << vcl_endl
            << "Iterations: " << minimizer.get_num_iterations() << "; "
            << "Evaluations: " << minimizer.get_num_evaluations() << vcl_endl;
    */
  }
  vcl_cout << "Solution: " << params << vcl_endl;
}


int gmmreg_rigid::set_init_params() {
  char f_init_affine[80]={0}, f_init_rigid[80]={0};
  set_init_rigid(f_init_rigid);
  //param_all.set_size(n,d);
  return 0;
}

int gmmreg_rigid::set_init_params(const char* f_config) {
  char f_init_affine[80]={0}, f_init_rigid[80]={0};
  GetPrivateProfileString(section, "init_rigid", NULL,
      f_init_rigid, 80, f_config);
  set_init_rigid(f_init_rigid);
  //param_all.set_size(n,d);
  return 0;
}

int gmmreg_rigid::set_init_rigid(const char* filename) {
  vnl_random random;
  assert((d==2)||(d==3));
  if (d==2) {
    param_rigid.set_size(3);
    param_rigid.fill(0);
    param_rigid[0] = 0; //random.drand32(-1,1);
    param_rigid[1] = 0; //random.drand32(-1,1);
    param_rigid[2] = 0; //random.drand32(-3.14,3.14);
    //vcl_cout << param_rigid << vcl_endl;
  } else if (d==3) {
    param_rigid.set_size(7);
    param_rigid.fill(0);
    param_rigid[3] = 1;
  }
  return 0;
}

void gmmreg_rigid::set_param(vnl_vector<double>& x0) {
  x0 = param_rigid;
}

void gmmreg_rigid::perform_transform(const vnl_vector<double> &x) {
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
    quaternion2rotation(q, rotation);
    translation.set_size(1,3);
    translation[0][0] = x[4];
    translation[0][1] = x[5];
    translation[0][2] = x[6];
  }
  transformed_model = model*rotation.transpose() +  ones*translation;
  param_rigid = x;  
  vnl_matrix<double> output_model = transformed_model;
  if (b_normalize)
    denormalize(output_model,model_centroid, model_scale);
  funcPub(output_model);
}

void gmmreg_rigid::save_results(const char* f_config,
    const vnl_vector<double>& params) {
  char f_transformed[80] = {0};
  char f_final_rigid[80] = {0};

  GetPrivateProfileString(common_section, "transformed_model",
      NULL, f_transformed, 80, f_config);
  save_transformed( f_transformed, params, f_config );

  GetPrivateProfileString(common_section, "final_rigid",
      NULL, f_final_rigid, 80, f_config);
  save_vector(f_final_rigid, param_rigid);
}

void gmmreg_rigid::prepare_own_options(int level, char* sigma, char* max_function_evals)
{
    multi_scale_options(level, sigma, max_function_evals);
}
void gmmreg_rigid::prepare_own_options(const char* f_config) {
  multi_scale_options(f_config);
}
