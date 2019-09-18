#ifndef _WB_SOT_SOLVERS_QP_OASES_BE_H_
#define _WB_SOT_SOLVERS_QP_OASES_BE_H_

#include <OpenSoT/solvers/BackEnd.h>
#include <boost/shared_ptr.hpp>

#define QPOASES_DEFAULT_EPS_REGULARISATION 2E2

namespace qpOASES {
    class SQProblem;
    class Options;
    class Bounds;
    class Constraints;
}

namespace OpenSoT{
    namespace solvers{

    /**
     * @brief The QPOasesBackEnd class handle variables, options and execution of a
     * single qpOases problem. Is implemented using Eigen.
     * This represent the Back-End.
     */
    class QPOasesBackEnd:  public BackEnd{
    public:
        /**
         * @brief QPOasesBackEnd constructor with creation of a QP problem.
         * @param number_of_variables of the QP problem
         * @param number_of_constraints of the QP problem
         * @param hessian_type of the QP problem
         * @param eps_regularization set the Scaling factor of identity matrix used for Hessian regularisation.
         *             final_eps_regularisation = standard_eps_regularisation * eps_regularisation
         *        this parameter is particular important for the optimization!
         */
        QPOasesBackEnd(const int number_of_variables,
                       const int number_of_constraints,
                       OpenSoT::HessianType hessian_type = OpenSoT::HST_UNKNOWN,
                       const double eps_regularisation = QPOASES_DEFAULT_EPS_REGULARISATION); //2E2

        /**
          * @brief ~QPOasesBackEnd destructor
          */
        ~QPOasesBackEnd();

        /**
         * @brief setDefaultOptions to internal qpOases problem.
         * Default are set to:
         *  opt.setToMPC();
         *  opt.printLevel = qpOASES::PL_NONE;
         *  opt.enableRegularisation = qpOASES::BT_TRUE;
         *  opt.epsRegularisation *= _epsRegularisation;
         *  opt.numRegularisationSteps = 2;
         *  opt.numRefinementSteps = 1;
         *  opt.enableFlippingBounds = qpOASES::BT_TRUE;
         *
         */
        void setDefaultOptions();

        /**
         * @brief getProblem return the internal QP problem
         * @return reference to internal QP problem
         */
        const boost::shared_ptr<qpOASES::SQProblem>& getProblem(){return _problem;}

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
        virtual bool initProblem(const Eigen::MatrixXd& H, const Eigen::VectorXd& g,
                        const Eigen::MatrixXd& A,
                        const Eigen::VectorXd& lA, const Eigen::VectorXd& uA,
                        const Eigen::VectorXd& l, const Eigen::VectorXd& u);

        /**
         * This set of function update current problem copying input data. Use these
         * methods to update existing matrices of the QP problem.
         */

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
                               const Eigen::Ref<const Eigen::VectorXd> &lA, 
                               const Eigen::Ref<const Eigen::VectorXd> &uA);


        /**
         * @brief solve the QP problem
         * @return true if the QP problem is solved
         */
        virtual bool solve();


        /**
         * @brief getHessianType return the hessian type f the problem
         * @return hessian type
         */
        OpenSoT::HessianType getHessianType();

        /**
         * @brief setHessianType of the problem
         * @param ht hessian type
         */
        void setHessianType(const OpenSoT::HessianType ht);

        /**
         * @brief getnWSR return maximum number of working set recalculations
         * @return maximum number of working set recalculations
         */
        int getnWSR(){return _nWSR;}

        /**
         * @brief setnWSR set maximum number of working set recalculations
         * @param nWSR Maximum number of working set recalculations
         */
        void setnWSR(const int nWSR){_nWSR = nWSR;}

        /**
         * @brief getActiveBounds return the active bounds of the solved QP problem
         * @return active bounds
         */
        const qpOASES::Bounds& getActiveBounds(){return *_bounds;}

        /**
         * @brief getActiveConstraints return the active constraints of the solved QP problem
         * @return active constraints
         */
        const qpOASES::Constraints& getActiveConstraints(){return *_constraints;}


        /**
         * @brief printProblemInformation print some extra information about the problem
         * @param problem_number a number to identify the problem
         * @param problem_id a string to identify the problem
         * @param constraints_id a string to identify the constraints associated to the problem
         * @param bounds_id a string to identify the bounds associated to the problem
         */
        virtual void _printProblemInformation();

        /**
         * @brief getObjective return the value of the cost function at the optimum
         * @return finite value: Optimal objective function value (QP was solved)
         *            +infinity: QP was not yet solved
         */
        virtual double getObjective();

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
            return _epsRegularisation;
        }

    protected:
        /**
         * @brief printConstraintsInfo function that print informations when the problem is not feasible
         */
        void printConstraintsInfo();

        /**
         * @brief checkINFTY if a bound/constraint is set to a value less than -INFTY then the bound/constraint is
         * set to -INFTY, if a bound/constraint is set to a value more than INFTY then the bound/constraint is
         * set to INFTY.
         */
        void checkINFTY();

        /**
         * @brief _problem is the internal SQProblem
         */
        boost::shared_ptr<qpOASES::SQProblem> _problem;

        /**
         * @brief _bounds are the active bounds of the SQProblem
         */
        boost::shared_ptr<qpOASES::Bounds> _bounds;

        /**
         * @brief _constraints are the active constraints of the SQProblem
         */
        boost::shared_ptr<qpOASES::Constraints> _constraints;

        /**
         * @brief _nWSR is the maximum number of working set recalculations
         */
        int _nWSR;

        /**
         * @brief _epsRegularisation is a factor that multiplies standard epsRegularisation of qpOases
         */
        double _epsRegularisation;


        /**
         * @brief _opt solver options
         */
        boost::shared_ptr<qpOASES::Options> _opt;

        /**
         * Solution of the QP problem
         */
        Eigen::VectorXd _dual_solution;
        
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> _A_rm;

    };
    }
}
#endif
