/*=========================================================================
$Author: bing.jian@gmail.com $
$Date: 2010-05-10 07:32:09 +0200 (Mon, 10 May 2010) $
$Revision: 127 $
=========================================================================*/

/*=========================================================================
$Modified by Koosy (koosyong@gmail.com)
$Date: 2012-06-16
$Revision: Interface for passing model/scene data directly from function calls$
=========================================================================*/

/** 
 * \file gmmreg_base.h
 * \brief  The declaration of the base class
 */


#ifndef gmmreg_base_h
#define gmmreg_base_h

#ifdef WIN32
#include <windows.h>
#else
#include "port_ini.h"
#endif

#include <vector>
#include <vnl/vnl_vector.h>
#include <vnl/vnl_matrix.h>



/** 
 * \class gmmreg_base
 * \brief  the base class
 */
class gmmreg_base{

public:
    gmmreg_base(){strcpy(common_section,"FILES");}
    int m,n,s,d;
    /* m: points in model */
    /* s: points in scene */
    /* n: points in ctrl_pts */
    /* d: dimensionality  2D or 3D */

    vnl_matrix<double> model, scene, ctrl_pts, transformed_model; 
    /* each row is a sample point */

    void run();
    void run(const char* f_config);
    void init(vnl_matrix<double>& _model, vnl_matrix<double>& _scene, vnl_matrix<double>& _ctrl_pts, bool _normalize);

    virtual void perform_transform(const vnl_vector<double>&) = 0;  
    virtual double bending_energy() = 0; //serving as a regularization term
    virtual void compute_gradient(double lambda, const vnl_matrix<double>& gradient, vnl_matrix<double>& grad_all) = 0;

    virtual ~gmmreg_base(){}

protected:
    double sigma, lambda; 
    vnl_matrix<double> kernel;
    int b_normalize;
    vnl_vector<double> model_centroid, scene_centroid;
    char section[80], common_section[80];

    unsigned int level;
    std::vector<double> v_scale; 
    std::vector<int> v_func_evals;

    virtual int prepare_input(const char* input_config); // load input data from files
    int set_model(const char* filename);
    int set_scene(const char* filename);
    int set_ctrl_pts(const char* filename);
    void save_transformed(const char* filename, const vnl_vector<double>&,const char* f_config);
    void multi_scale_options(const char* f_config);
    void multi_scale_options(int _level, char* _sigma, char* _max_function_evals);

protected:
    double model_scale, scene_scale;
    void prepare_common_options(const char* f_config);
    void prepare_common_options(bool _normalize);
    virtual void prepare_own_options(const char* f_config) = 0;
    virtual void prepare_basis_kernel() = 0;
    virtual int set_init_params(const char* filename) = 0;
    virtual void save_results(const char* filename, const vnl_vector<double>&) = 0;
    virtual void start_registration(vnl_vector<double>& params) = 0;
    int initialize(const char* f_config);

};


#endif //#ifndef gmmreg_base_h
