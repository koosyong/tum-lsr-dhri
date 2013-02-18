#ifndef PCGMMREG_H
#define PCGMMREG_H

#include "pcobject.h"
#include <vnl/vnl_random.h>
#include <vector>
#include <vnl/vnl_vector.h>
#include <vnl/vnl_matrix.h>

typedef void(*FuncPub)(vnl_matrix<double> _transformed);

class PCGMMReg_func;

class PCGMMReg
{
public:


    PCGMMReg();
    ~PCGMMReg(){
        delete func;
        func = NULL;
    }

    void init(shared_ptr<PCObject> _model, shared_ptr<PCObject> _scene, int _maxEval, FuncPub _funcPub = 0);
    void run();
    void perform_transform(const vnl_vector<double> &x);

private:
    void start_registration(vnl_vector<double>& params);
    void start_registration2(vnl_vector<double>& params);
    void set_bound();
    void translate(shared_ptr<PCObject> object, vnl_vector<double> param);

public:
    PCObject transformed_model;
    int m,n,s,d;
    /* m: points in model */
    /* s: points in scene */
    /* n: points in ctrl_pts */
    /* d: dimensionality  2D or 3D */

//    vnl_vector<double> params;
    vnl_matrix<double> modelpoints, scenepoints;
    vnl_vector<double> param_norm;

    shared_ptr<PCObject> model;
    shared_ptr<PCObject> scene;

public:
    FuncPub funcPub;
    PCGMMReg_func *func;
    vnl_vector<double> param_rigid;
    vnl_vector<long> nbd;
    vnl_vector<double> lower_bound, upper_bound;

    int maxEval;

};

#endif // PCGMMREG_H
