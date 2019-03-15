#ifndef _WB_SOT_SOLVERS_UQUADPROG_BE_H_
#define _WB_SOT_SOLVERS_UQUADPROG_BE_H_

#include <OpenSoT/solvers/BackEnd.h>
#include <boost/shared_ptr.hpp>
#include <OpenSoT/utils/Piler.h>

#define EIGQUADPROG

#ifdef UQUADPROG
    #include <uquadprog.hpp>
#endif

#ifdef EIGQUADPROG
    #include <eiquadprog.hpp>
#endif

#define UQUADPROG_DEFAULT_EPS_REGULARISATION 0

using namespace OpenSoT::utils;

namespace OpenSoT{
namespace solvers{

class uQuadProgBackEnd:  public BackEnd{
public:
    typedef MatrixPiler VectorPiler;

    uQuadProgBackEnd(const int number_of_variables,
                     const int number_of_constraints,
                     const double eps_regularisation = UQUADPROG_DEFAULT_EPS_REGULARISATION);

    ~uQuadProgBackEnd();

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

#ifdef UQUADPROG
    //These are for the cost function: 0.5 * x G x + g0 x
    boost::numeric::ublas::matrix<double> _G;
    boost::numeric::ublas::vector<double> _g0;

    //These are for equality constraints: CE^T x + ce0 = 0,
    //in this implementation we consider all the constraints as inequalities
    boost::numeric::ublas::matrix<double> _CE;
    boost::numeric::ublas::vector<double> _ce0;

    //These are for inequality constraints: CE^T x + ce0 => 0
    boost::numeric::ublas::matrix<double> _CI;
    boost::numeric::ublas::vector<double> _ci0;

    //Solution
    boost::numeric::ublas::vector<double> _x;
#endif

    MatrixPiler _CIPiler;
    VectorPiler _ci0Piler;



    void __generate_data_struct();


    double _f_value;

};

}
}

#endif
