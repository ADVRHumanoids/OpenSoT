#ifndef _WB_SOT_SOLVERS_QPSWIFT_BE_H_
#define _WB_SOT_SOLVERS_QPSWIFT_BE_H_

#include <OpenSoT/solvers/BackEnd.h>
#include <memory>
#include <OpenSoT/Task.h>
#include <qpSWIFT/qpSWIFT.h>
#include <OpenSoT/utils/Piler.h>

#define QPSWIFT_DEFAULT_EPS_REGULARISATION 0

using namespace OpenSoT::utils;


namespace OpenSoT{
namespace solvers{

/**
 * @brief The qpSWIFTBackEnd class handle variables, options and execution of a
 * single qpSWIFT problem. Is implemented using Eigen.
 * This represent the Back-End.
 *
 * @note This implementation makes use of ONLY inequality constraints (EQUALITY ARE THREATED AS INEQUALITY)
 */
class qpSWIFTBackEnd:  public BackEnd{
public:
    qpSWIFTBackEnd(const int number_of_variables,
                   const int number_of_constraints,
                   const double eps_regularisation = QPSWIFT_DEFAULT_EPS_REGULARISATION);

    ~qpSWIFTBackEnd();

    virtual bool initProblem(const Eigen::MatrixXd &H, const Eigen::VectorXd &g,
                             const Eigen::MatrixXd &A,
                             const Eigen::VectorXd &lA, const Eigen::VectorXd &uA,
                             const Eigen::VectorXd &l, const Eigen::VectorXd &u);

    virtual bool solve();

    virtual boost::any getOptions();

    virtual void setOptions(const boost::any& options);

    virtual bool updateTask(const Eigen::MatrixXd& H, const Eigen::VectorXd& g);

    virtual bool updateConstraints(const Eigen::Ref<const Eigen::MatrixXd>& A,
                                const Eigen::Ref<const Eigen::VectorXd>& lA,
                                const Eigen::Ref<const Eigen::VectorXd>& uA);

    virtual bool updateBounds(const Eigen::VectorXd& l, const Eigen::VectorXd& u);

    virtual double getObjective();

    bool setEpsRegularisation(const double eps)
    {
        if(eps < 0.)
            return false;
        _eps_regularisation = eps;
        return true;
    }

    virtual double getEpsRegularisation()
    {
        return _eps_regularisation;
    }
private:
    typedef MatrixPiler VectorPiler;

    std::shared_ptr<QP> _qp; //qpSWIFT data structure

    double _eps_regularisation;

    MatrixPiler _AA;
    VectorPiler _b;
    std::vector<unsigned int> _equality_indices;

    MatrixPiler _G;
    VectorPiler _h;
    std::vector<unsigned int> _inequality_indices;

    Eigen::MatrixXd _I;






};

}
}

#endif
