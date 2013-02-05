/*=========================================================================
$Date: 2010-05-10 07:32:09 +0200 (Mon, 10 May 2010) $
$Revision: 127 $
=========================================================================*/

/**
 * \file gmmreg_tps.h
 * \brief  The declaration of the class gmmreg_tps
 */


#ifndef gmmreg_tps_h
#define gmmreg_tps_h

#include <vector>

#include "gmmreg_base.h"
#include "gmmreg_tps_func.h"


/**
 * \class gmmreg_tps_L2
 * \brief non-rigid registration using thin-plate-splines
 */

class gmmreg_tps: public gmmreg_base {

public:
    typedef void(*FuncPub)(vnl_matrix<double> _transformed);
    gmmreg_tps(FuncPub _funcPub = 0) {
        strcpy(section,"GMMREG_OPT");
        funcPub = _funcPub;
    }
    virtual ~gmmreg_tps() {
        delete func;
    }

    void tpsInit(int _level, char* _sigma, char* _max_function_evals, char* _lamda, char* _fix_affine);

protected:
    gmmreg_tps_func *func;

private:
    vnl_matrix<double> param_affine, param_tps;
    vnl_matrix<double> after_tps, basis, param_all;
    //unsigned int level;
    //std::vector<int> v_func_evals;
    std::vector<double> v_lambda;
    std::vector<int> v_affine;

    FuncPub funcPub;

    void start_registration(vnl_vector<double>&);
    int set_init_affine(const char* filename);
    int set_init_tps(const char* filename);
    void set_param(vnl_vector<double>& x0);
    void set_affine_and_tps(const vnl_vector<double>&);
    int set_init_params(const char* filename);
    int set_init_params();
    void save_results(const char* f_config, const vnl_vector<double>&);

    int prepare_input(const char* input_config);
    void prepare_basis_kernel();
    void prepare_param_gradient(bool);
    void perform_transform(const vnl_vector<double>&);
    double bending_energy();
    void compute_gradient(double lambda, const vnl_matrix<double>& gradient, vnl_matrix<double>& grad_all);
    void prepare_own_options(const char* f_config);
    void prepare_own_options(int level, char* sigma, char* max_function_evals, char* lamda, char* fix_affine);
//    void prepare_own_options(char* lamda, char* fix_affine);

};

class gmmreg_tps_L2: public gmmreg_tps {
public:

    gmmreg_tps_L2(FuncPub _funcPub = 0): gmmreg_tps(_funcPub) {
        func = new gmmreg_tps_L2_func;        
    }


};

class gmmreg_tps_KC: public gmmreg_tps {
public:
    gmmreg_tps_KC(FuncPub _funcPub = 0): gmmreg_tps(_funcPub) {
        func = new gmmreg_tps_KC_func;
    }
};

#endif //#ifndef gmmreg_tps_h

