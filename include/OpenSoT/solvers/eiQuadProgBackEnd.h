#ifndef _WB_SOT_SOLVERS_UQUADPROG_BE_H_
#define _WB_SOT_SOLVERS_UQUADPROG_BE_H_

#include <OpenSoT/solvers/BackEnd.h>
#include <boost/shared_ptr.hpp>
#include <OpenSoT/utils/Piler.h>
#include <eiquadprog.hpp>

#define EIQUADPROG_DEFAULT_EPS_REGULARISATION 0

using namespace OpenSoT::utils;

namespace OpenSoT{
namespace solvers{

class eiQuadProgBackEnd:  public BackEnd{
public:
    typedef MatrixPiler VectorPiler;

    eiQuadProgBackEnd(const int number_of_variables,
                     const int number_of_constraints,
                     const double eps_regularisation = EIQUADPROG_DEFAULT_EPS_REGULARISATION);

    ~eiQuadProgBackEnd();

    virtual bool initProblem(const Eigen::MatrixXd &H, const Eigen::VectorXd &g,
                             const Eigen::MatrixXd &A,
                             const Eigen::VectorXd &lA, const Eigen::VectorXd &uA,
                             const Eigen::VectorXd &l, const Eigen::VectorXd &u);

    virtual bool solve();

    virtual double getObjective();

    virtual boost::any getOptions();

    virtual void setOptions(const boost::any& options);



private:
    double _eps_regularisation;
    Eigen::MatrixXd _I;


    MatrixPiler _CIPiler;
    VectorPiler _ci0Piler;



    void __generate_data_struct();


    double _f_value;

};

}
}

#endif
