#ifndef _WB_SOT_SOLVERS_CBC_BE_H_
#define _WB_SOT_SOLVERS_CBC_BE_H_

#include <OpenSoT/solvers/BackEnd.h>
#include <coin/CbcModel.hpp>
#include <coin/OsiCbcSolverInterface.hpp>
#include <coin/CbcOrClpParam.hpp>
#include <boost/shared_ptr.hpp>
#include <Eigen/Sparse>

namespace OpenSoT{
namespace solvers{
 
    class CBCBackEnd:  public BackEnd{
        public:
            CBCBackEnd(const int number_of_variables,
                    const int number_of_constraints,
                    const double eps_regularisation = 0.0);
            
            ~CBCBackEnd();
            
            /**
             * H NOT USED!
             * g Linear cost function: g'x
             * A Linear Constraints Ax
             * lA/uA constraints
             * l/u bounds
             **/ 
            virtual bool initProblem(const Eigen::MatrixXd &H, const Eigen::VectorXd &g,
                                     const Eigen::MatrixXd &A,
                                     const Eigen::VectorXd &lA, const Eigen::VectorXd &uA,
                                     const Eigen::VectorXd &l, const Eigen::VectorXd &u);
            /**
             * @brief setOptions
             * @param options
             * NOTE: THIS IS NOT RT SAFE!
             */
            virtual void setOptions(const boost::any& options);
            boost::any getOptions();
            bool solve();
            
            double getObjective();
            
            bool updateConstraints(const Eigen::Ref<const Eigen::MatrixXd>& A, 
                                const Eigen::Ref<const Eigen::VectorXd>& lA, 
                                const Eigen::Ref<const Eigen::VectorXd>& uA);
            
            struct CBCBackEndOptions
            {
                std::vector<int> integer_ind;
            };

            virtual void _printProblemInformation();

            bool solverReturnError();

            
        private:
            
            boost::shared_ptr<CbcModel> _model;          
            
            CoinPackedMatrix _ACP;
            Eigen::SparseMatrix<double> _AS;
            
            CBCBackEndOptions _opt;
            
            void __generate_data_struct(const int number_of_variables, const int number_of_constraints);

            std::vector<int> _integer_variables;

            Eigen::MatrixXd __H = _H;
            Eigen::VectorXd __g = _g;
            Eigen::MatrixXd __A = _A;
            Eigen::VectorXd __lA = _lA;
            Eigen::VectorXd __uA = _uA;
            Eigen::VectorXd __l = _l;
            Eigen::VectorXd __u = _u;
            
    };
    
}
}

#endif
