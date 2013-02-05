/*=========================================================================
$Date: 2010/02/24 04:51:29 $
$Revision: 1.1 $
=========================================================================*/

/**
 * \file gmmreg_grbf.h
 * \brief  The declaration of the class gmmreg_grbf
 */


#ifndef gmmreg_grbf_h
#define gmmreg_grbf_h

#include <vector>

#include "gmmreg_base.h"
#include "gmmreg_grbf_func.h"

/**
 * \class gmmreg_grbf_L2
 * \brief non-rigid registration using thin-plate-splines
 */

class gmmreg_grbf: public gmmreg_base {

public:
    typedef void(*FuncPub)(vnl_matrix<double> _transformed);
    gmmreg_grbf(FuncPub _funcPub = 0) {
        strcpy(section,"GMMREG_OPT");
        funcPub = _funcPub;
    }
    virtual ~gmmreg_grbf() {
        delete func;
    }

    void grbfInit(int _level, char* _sigma, char* _max_function_evals, char* _lamda, char* _fix_affine);

protected:
    gmmreg_grbf_func *func;

private:
    vnl_matrix<double> param_grbf;
    vnl_matrix<double> after_grbf, basis, param_all;
    //unsigned int level;
    //std::vector<int> v_func_evals;
    std::vector<double> v_beta;
    double beta;
    std::vector<double> v_lambda;
    std::vector<int> v_affine;

    FuncPub funcPub;

    void start_registration(vnl_vector<double>&);
    int set_init_grbf(const char* filename);
    void set_param(vnl_vector<double>& x0);
    void set_grbf(const vnl_vector<double>&);
    int set_init_params(const char* filename);
    void save_results(const char* f_config, const vnl_vector<double>&);

    int prepare_input(const char* input_config);
    void prepare_basis_kernel();
    void prepare_param_gradient(bool);
    void perform_transform(const vnl_vector<double>&);
    double bending_energy();
    void compute_gradient(double lambda, const vnl_matrix<double>& gradient, vnl_matrix<double>& grad_all);
    void prepare_own_options(const char* f_config);
    void prepare_own_options(int level, char* sigma, char* max_function_evals, char* lamda, char* fix_affine);

};

class gmmreg_grbf_L2: public gmmreg_grbf {
public:
    gmmreg_grbf_L2(FuncPub _funcPub = 0): gmmreg_grbf(_funcPub) {
        func = new gmmreg_grbf_L2_func;
    }
};

class gmmreg_grbf_KC: public gmmreg_grbf {
public:
    gmmreg_grbf_KC(): gmmreg_grbf() {
        func = new gmmreg_grbf_KC_func;
    }
};

#endif //#ifndef gmmreg_grbf_h

