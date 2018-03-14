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
    
class OSQPBackEnd:  public BackEnd{
    
public:
    
    typedef MatrixPiler VectorPiler;

    OSQPBackEnd(const int number_of_variables,
                const int number_of_constraints,
                const double eps_regularisation = OSQP_DEFAULT_EPS_REGULARISATION);
    
    ~OSQPBackEnd();

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

private:
    
    typedef Eigen::SparseMatrix<double> SparseMatrix;
    typedef Eigen::SparseMatrix<double, Eigen::RowMajor> SparseMatrixRowMajor;
    
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
    Eigen::MatrixXd _Pdense;
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
