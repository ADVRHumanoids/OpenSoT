#ifndef _WB_SOT_SOLVERS_OSQP_BE_H_
#define _WB_SOT_SOLVERS_OSQP_BE_H_

#include <OpenSoT/solvers/BackEnd.h>
#include <boost/shared_ptr.hpp>
#include <OpenSoT/Task.h>
#include <osqp/osqp.h>
#include <OpenSoT/utils/Piler.h>
#include <Eigen/Sparse>

#define OSQP_DEFAULT_EPS_REGULARISATION 0

using namespace OpenSoT::utils;

namespace OpenSoT{
namespace solvers{

/**
 * @brief The OSQPBackEnd class handle variables, options and execution of a
 * single osqp problem. Is implemented using Eigen.
 * This represent the Back-End.
 */
class OSQPBackEnd:  public BackEnd{
    
public:
    
    typedef MatrixPiler VectorPiler;

    /**
     * @brief OSQPBackEnd constructor with creation of a QP problem.
     * @param number_of_variables of the QP problem
     * @param number_of_constraints of the QP problem
     * @param eps_regularisation set the Scaling factor of identity matrix used for Hessian regularisation.s
     */
    OSQPBackEnd(const int number_of_variables,
                const int number_of_constraints,
                const double eps_regularisation = OSQP_DEFAULT_EPS_REGULARISATION);
    
    /**
      * @brief ~OSQPBackEnd destructor
      */
    ~OSQPBackEnd();

    /**
     * @brief initProblem initialize the QP problem and get the solution, the dual solution,
     * bounds and constraints.
     * The QP problem has the following structure:
     *
     *      min = ||Hx - g||
     *  st.     lA <= Ax <= uA
     *           l <=  x <= u
     * @param H Task Matrix
     * @param g Task references
     * @param A Constraint Matrix
     * @param lA lower constraint Eigen::VectorXd
     * @param uA upper constraint Eigen::VectorXd
     * @param l lower bounds
     * @param u upper bounds
     * @return true if the problem can be solved
     */
    virtual bool initProblem(const Eigen::MatrixXd &H, const Eigen::VectorXd &g,
                             const Eigen::MatrixXd &A,
                             const Eigen::VectorXd &lA, const Eigen::VectorXd &uA,
                             const Eigen::VectorXd &l, const Eigen::VectorXd &u);

    /**
     * @brief solve the QP problem
     * @return true if the QP problem is solved
     */
    virtual bool solve();

    /**
     * @brief getOptions return the options of the QP problem
     * @return options
     */
    virtual boost::any getOptions();

    /**
     * @brief setOptions of the QP problem.
     * @param options
     */
    virtual void setOptions(const boost::any& options);

    /**
     * @brief updateTask update internal H and g:
     * _H = H
     * _g = g
     * for now is not possible to have different size of H and g wrt internal ones
     * @param H updated task matrix
     * @param g updated reference Eigen::VectorXd
     * @return true if task is correctly updated
     */
    virtual bool updateTask(const Eigen::MatrixXd& H, const Eigen::VectorXd& g);

    /**
     * @brief updateConstraints update internal A, lA and uA
     * _A = A
     * _lA = lA
     * _uA = uA
     * A, lA and uA can change rows size to allow variable constraints
     * @param A update constraint matrix
     * @param lA update lower constraint Eigen::VectorXd
     * @param uA update upper constraint Eigen::VectorXd
     * @return true if constraints are correctly updated
     */
    virtual bool updateConstraints(const Eigen::Ref<const Eigen::MatrixXd>& A, 
                                const Eigen::Ref<const Eigen::VectorXd>& lA, 
                                const Eigen::Ref<const Eigen::VectorXd>& uA);

    /**
     * @brief updateBounds update internal l and u
     * _l = l
     * _u = u
     * @param l update lower bounds
     * @param u update upper bounds
     * @return true if bounds are correctly updated
     */
    virtual bool updateBounds(const Eigen::VectorXd& l, const Eigen::VectorXd& u);

    /**
     * @brief getObjective to retrieve the value of the objective function
     * @return the value of the objective function at the optimum
     */
    virtual double getObjective();

private:
    
    typedef Eigen::SparseMatrix<double> SparseMatrix;
    typedef Eigen::SparseMatrix<double, Eigen::RowMajor> SparseMatrixRowMajor;
    
    /**
     * @brief __generate_data_struct creates DENSE Hessian and Constraints matrices using a SPARSE representation.
     * Note that bounds are treated as constraints.
     * @param number_of_variables of the QP
     * @param number_of_constraints of the QP
     * @param number_of_bounds of the QP
     */
    void __generate_data_struct(const int number_of_variables, const int number_of_constraints, const int number_of_bounds);
    void update_data_struct();
    
    void upper_triangular_sparse_update();
    
    
    /**
    * @brief _problem is the internal OSQPWorkspace
    */
    boost::shared_ptr<OSQPWorkspace> _workspace;

    /**
    * @brief _data internal OSQPData
    */
    boost::shared_ptr<OSQPData> _data;

    /**
    * @brief _settings internal OSQPSettings
    */
    boost::shared_ptr<OSQPSettings> _settings;

    Eigen::VectorXd _lb_piled, _ub_piled;

    SparseMatrix _Asparse, _Asparse_upper;
    SparseMatrixRowMajor _Asparse_rowmaj;
    SparseMatrix _Psparse;
    Eigen::MatrixXd _Adense;
    Eigen::VectorXd _P_values;

    Eigen::MatrixXd _eye;

    boost::shared_ptr<csc> _Acsc;
    boost::shared_ptr<csc> _Pcsc;

    double _eps_regularisation;
    Eigen::VectorXd _I;



//        void print_csc_matrix_raw(csc* a, const std::string& name);

    void setCSCMatrix(csc* a, SparseMatrix& A);

};
}
}

#endif
