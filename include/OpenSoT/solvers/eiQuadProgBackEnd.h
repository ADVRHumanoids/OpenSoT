#ifndef _WB_SOT_SOLVERS_UQUADPROG_BE_H_
#define _WB_SOT_SOLVERS_UQUADPROG_BE_H_

#include <OpenSoT/solvers/BackEnd.h>
#include <memory>
#include <OpenSoT/utils/Piler.h>
#include <eiquadprog.hpp>

#define EIQUADPROG_DEFAULT_EPS_REGULARISATION 0

using namespace OpenSoT::utils;

namespace OpenSoT{
namespace solvers{

/**
 * @brief The eiQuadProgBackEnd class implements a back-end based on eiQuadProg by B. Stepehn https://www.cs.cmu.edu/~bstephe1/eiquadprog.hpp
 */
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

    /**
     * @brief setEpsRegularisation OVERWRITES the actual eps regularisation factor
     * @param eps the new regularisation factor
     * @return false if eps < 0
     */
    bool setEpsRegularisation(const double eps);

    /**
     * @brief getEpsRegularisation return internal solver eps
     * @return eps value
     */
    virtual double getEpsRegularisation()
    {
        return _eps_regularisation;
    }

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
