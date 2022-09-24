#ifndef _WB_SOT_SOLVERS_PROXQP_BE_H_
#define _WB_SOT_SOLVERS_PROXQP_BE_H_

#include <OpenSoT/solvers/BackEnd.h>
#include <memory>
#include <OpenSoT/Task.h>
#include <proxsuite/proxqp/dense/dense.hpp>
#include <OpenSoT/utils/Piler.h>

#define PROXQP_DEFAULT_EPS_REGULARISATION 0

using namespace OpenSoT::utils;
using namespace proxsuite;
using namespace proxsuite::proxqp;


namespace OpenSoT{
namespace solvers{

class proxQPBackEnd:  public BackEnd{
public:
    proxQPBackEnd(const int number_of_variables,
                  const int number_of_constraints,
                  const double eps_regularisation = PROXQP_DEFAULT_EPS_REGULARISATION);

    ~proxQPBackEnd();

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
    void create_data_structure(const Eigen::MatrixXd &A, const Eigen::VectorXd &lA, const Eigen::VectorXd &uA,
                               const Eigen::VectorXd &l, const Eigen::VectorXd &u);

    typedef MatrixPiler VectorPiler;

    std::shared_ptr<dense::QP<double>> _QP;

    MatrixPiler _AA; //equality constr
    VectorPiler _b;
    std::vector<unsigned int> _equality_constraint_indices;
    std::vector<unsigned int> _equality_bounds_indices;

    MatrixPiler _G; //inequality constr
    VectorPiler _ll, _uu;
    std::vector<unsigned int> _inequality_constraint_indices;
    std::vector<unsigned int> _inequality_bounds_indices;

    Eigen::MatrixXd _I;

    double _eps_regularisation;


};

}
}

#endif
