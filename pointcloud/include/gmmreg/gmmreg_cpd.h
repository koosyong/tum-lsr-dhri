/*=========================================================================
$Author: bing.jian@gmail.com $
$Date: 2010-05-10 07:32:09 +0200 (Mon, 10 May 2010) $
$Revision: 127 $
=========================================================================*/

/** 
 * \file gmmreg_cpd.h
 * \brief  The declaration of the class gmmreg_cpd
 */


#ifndef gmmreg_cpd_h
#define gmmreg_cpd_h


#include "gmmreg_base.h"

/** 
 * \class gmmreg_cpd
 * \brief  non-rigid registration by coherent point drifting
 */

class gmmreg_cpd : public gmmreg_base {

public:
    typedef void(*FuncPub)(vnl_matrix<double> _transformed);
    gmmreg_cpd(FuncPub _funcPub = 0): eps(0.0000000001) {
        strcpy(section,"GMMREG_EM");
        funcPub = _funcPub;
    }
    virtual ~gmmreg_cpd() {}
    void cpdInit();

private:    
    FuncPub funcPub;

    void start_registration(vnl_vector<double>&);
    void set_param(vnl_vector<double>&);
    int set_init_params(const char*);
    int set_init_params();
    void save_results(const char*, const vnl_vector<double>&);
    int prepare_input(const char*);
    void prepare_param_gradient(bool);
    void perform_transform(const vnl_vector<double>&);
    double bending_energy();
    void compute_gradient(double lambda, const vnl_matrix<double>& gradient, vnl_matrix<double>& grad_all){};
    void prepare_own_options(const char*);
    void prepare_own_options();
    virtual void prepare_basis_kernel() = 0;
    virtual double update_param() = 0;

protected:
    vnl_matrix<double> basis, param_all;
    double EMtol, tol, beta, anneal;
    int max_iter, max_em_iter, outliers;
    //vnl_vector<double> column_sum;
    //double outlier_term;
    vnl_matrix<double> P;
    double eps; 

};


class gmmreg_cpd_tps: public gmmreg_cpd {
private:
    vnl_matrix<double> tps;
    vnl_matrix<double> affine;
    vnl_matrix<double> nP;
    vnl_matrix<double> G,Q1,Q2,R,invR,invG;

    void prepare_basis_kernel();
    double update_param();
};

class gmmreg_cpd_grbf: public gmmreg_cpd {
public:
    gmmreg_cpd_grbf(FuncPub _funcPub = 0): gmmreg_cpd(_funcPub){}

private:
    vnl_matrix<double> dPG;
    vnl_matrix<double> dPY0;
    vnl_matrix<double> Gtranspose, invG;

    void prepare_basis_kernel();
    double update_param();
};


#endif //#ifndef gmmreg_cpd_h
