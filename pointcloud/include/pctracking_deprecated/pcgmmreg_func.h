#ifndef PCGMMREG_FUNC_H
#define PCGMMREG_FUNC_H

#include <vnl/vnl_cost_function.h>
#include "pcgmmreg.h"

//class PCGMMReg;

class PCGMMReg_func : public vnl_cost_function
{
public:
    PCGMMReg_func(): vnl_cost_function(){}

    double eval(double &, vnl_matrix<double> &);
    double f(const vnl_vector<double>& x);
    void gradf(const vnl_vector<double>& x, vnl_vector<double>& g);

    PCGMMReg* gmmreg;
    inline void set_gmmreg(PCGMMReg* gmmreg){
        this->gmmreg = gmmreg;
        this->m = gmmreg->m;
        this->d = gmmreg->d;
        this->s = gmmreg->s;
        if (d==2){
            set_number_of_unknowns(3);
        }
        else if (d==3){
            set_number_of_unknowns(7);
        }
        gradient.set_size(m,d);

    }

private:
    double weight_l2(PCObject &o1, PCObject &o2);

protected:
    vnl_matrix<double> gradient;

private:
    double scale, lambda;
    int m,d,s;

};

#endif // PCGMMREG_FUNC_H
