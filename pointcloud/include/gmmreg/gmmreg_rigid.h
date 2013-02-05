/*=========================================================================
$Author: bing.jian $
$Date: 2009-02-10 08:13:49 +0100 (Tue, 10 Feb 2009) $
$Revision: 121 $
=========================================================================*/

/**
 * \file gmmreg_rigid.h
 * \brief  The declaration of the class gmmreg_rigid
 */


#ifndef gmmreg_rigid_h
#define gmmreg_rigid_h

#include <assert.h>

#include <iostream>
#include <fstream>


#include <vnl/vnl_random.h>
#include "gmmreg_base.h"
#include "gmmreg_rigid_func.h"

/**
 * \class gmmreg_rigid
 * \brief  rigid registration by minimizing L2 distance or maximizing KC
 */

class gmmreg_rigid: public gmmreg_base {

public:
    typedef void(*FuncPub)(vnl_matrix<double> _transformed);
    gmmreg_rigid(FuncPub _funcPub = 0) {
        strcpy(section,"GMMREG_OPT");
        func = new gmmreg_rigid_func;
        funcPub = _funcPub;
    }
    ~gmmreg_rigid(){
        delete func;
    }

    void rigidInit(int _level, char* _sigma, char* _max_function_evals);

private:
    vnl_vector<double> param_rigid;
    gmmreg_rigid_func *func;
    vnl_vector<long> nbd;
    vnl_vector<double> lower_bound, upper_bound;

    FuncPub funcPub;

    void set_bound();
    void start_registration(vnl_vector<double>&);
    int set_init_rigid(const char*);
    void set_param(vnl_vector<double>&);
    int set_init_params(const char*);
    int set_init_params();
    void save_results(const char*, const vnl_vector<double>&);
    void prepare_basis_kernel() {};
    void prepare_param_gradient(bool){};
    void perform_transform(const vnl_vector<double>&);
    double bending_energy() {return 0;};
    void compute_gradient(double lambda, const vnl_matrix<double>& gradient, vnl_matrix<double>& grad_all){};
    void prepare_own_options(const char*);
    void prepare_own_options(int level, char* sigma, char* max_function_evals);

};

#endif //#ifndef gmmreg_rigid_h
