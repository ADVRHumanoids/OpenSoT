#ifndef _WB_SOT_SOLVERS_OSQP_BE_H_
#define _WB_SOT_SOLVERS_OSQP_BE_H_

#include <OpenSoT/solvers/BackEnd.h>
#include <boost/shared_ptr.hpp>
#include <OpenSoT/Task.h>
#include <osqp/osqp.h>
#include <OpenSoT/utils/Piler.h>
#include <Eigen/Sparse>

using namespace OpenSoT::utils;

namespace OpenSoT{
    namespace solvers{
    class OSQPBackEnd:  public BackEnd{
    public:
          typedef MatrixPiler VectorPiler;

          OSQPBackEnd(const int number_of_variables,
                      const int number_of_constraints);
          ~OSQPBackEnd();

          virtual bool initProblem(const Eigen::MatrixXd &H, const Eigen::VectorXd &g,
                                                   const Eigen::MatrixXd &A,
                                                   const Eigen::VectorXd &lA, const Eigen::VectorXd &uA,
                                                   const Eigen::VectorXd &l, const Eigen::VectorXd &u);

          virtual bool solve();

          virtual boost::any getOptions();

          virtual void setOptions(const boost::any& options);

    private:
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

        MatrixPiler _Apiled;
        VectorPiler _lApiled;
        VectorPiler _uApiled;

        Eigen::SparseMatrix<double> _Asp;
        Eigen::SparseMatrix<double> _Psp;

        Eigen::MatrixXd I;

        boost::shared_ptr<csc> _Acsc;
        boost::shared_ptr<csc> _Pcsc;

        /**
         * @brief toData copies all the internal Eigen Matrices to OSQPData
         */
        void toData();

        void print_csc_matrix_raw(csc* a, const std::string& name);


    };
    }
}

#endif
