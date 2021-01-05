#ifndef _WB_SOT_SOLVERS_BACK_END_H_
#define _WB_SOT_SOLVERS_BACK_END_H_

#include <Eigen/Dense>
#include <XBotInterface/Logger.hpp>
#include <boost/any.hpp>
#include <OpenSoT/Task.h>

namespace OpenSoT{
    namespace solvers{

    class BackEnd{
    public:
        BackEnd(const int number_of_variables, const int number_of_constraints);
        virtual ~BackEnd();

        typedef boost::shared_ptr<BackEnd> Ptr;

        /**
         * @brief getSolution return the actual solution of the QP problem
         * @return solution
         */
        const Eigen::VectorXd& getSolution(){return _solution;}

        /**
         * Getters for internal matrices and Eigen::VectorXds
         */
        const Eigen::MatrixXd& getH(){return _H;}
        const Eigen::VectorXd& getg(){return _g;}
        const Eigen::MatrixXd& getA(){return _A;}
        const Eigen::VectorXd& getlA(){return _lA;}
        const Eigen::VectorXd& getuA(){return _uA;}
        const Eigen::VectorXd& getl(){return _l;}
        const Eigen::VectorXd& getu(){return _u;}
        
        int getNumVariables() const;
        int getNumConstraints() const;

        /**
         * @brief log Tasks, Constraints and Bounds matrices
         * @param logger a pointer to a MatLogger
         * @param i an index related to the particular index of the problem
         * @param prefix a prefix before the logged matrices
         */
        void log(XBot::MatLogger2::Ptr logger, int i, const std::string& prefix);

        /**
         * @brief updateProblem update the whole problem see updateTask(), updateConstraints() and updateBounds()
         * @param H updated task matrix
         * @param g updated reference Eigen::VectorXd
         * @param A update constraint matrix
         * @param lA update lower constraint Eigen::VectorXd
         * @param uA update upper constraint Eigen::VectorXd
         * @param l update lower bounds
         * @param u update upper bounds
         * @return if the problem is correctly updated
         */
        bool updateProblem(const Eigen::MatrixXd &H, const Eigen::VectorXd &g,
                                           const Eigen::MatrixXd &A, const Eigen::VectorXd &lA, const Eigen::VectorXd &uA,
                                           const Eigen::VectorXd &l, const Eigen::VectorXd &u);

        void printProblemInformation(const int problem_number, const std::string& problem_id,
                                     const std::string& constraints_id, const std::string& bounds_id);

        ///VIRTUAL METHODS

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
         * @brief updateBounds update internal l and u
         * _l = l
         * _u = u
         * @param l update lower bounds
         * @param u update upper bounds
         * @return true if bounds are correctly updated
         */
        virtual bool updateBounds(const Eigen::VectorXd& l, const Eigen::VectorXd& u);



        ///PURE VIRTUAL METHODS:

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
                                 const Eigen::MatrixXd& A, const Eigen::VectorXd& lA, const Eigen::VectorXd& uA,
                                 const Eigen::VectorXd& l, const Eigen::VectorXd& u) = 0;
        /**
         * @brief solve the QP problem
         * @return true if the QP problem is solved (implementation should use internally _solution)
         */
        virtual bool solve() = 0;

        /**
         * @brief getOptions return the options of the QP problem
         * @return options, a boost::any object since we do not know the type of the object which depends on the solver
         */
        virtual boost::any getOptions() = 0;

        /**
         * @brief setOptions of the QP problem.
         * @param options, a boost::any object since we do not know the type of the object which depends on the solver
         */
        virtual void setOptions(const boost::any& options) = 0;

        /**
         * @brief getObjective to retrieve the value of the objective function
         * @return the value of the objective function at the optimum
         */
        virtual double getObjective() = 0;

        /**
         * @brief setEpsRegularisation is used to set internal solver eps
         * @param eps regularisation
         * @return false by default
         */
        virtual bool setEpsRegularisation(const double eps)
        {
            XBot::Logger::error("BackEnd does not allow setEpsRegularisation");
            return false;
        }

        /**
         * @brief getEpsRegularisation return internal solver eps
         * @return eps value
         */
        virtual double getEpsRegularisation()
        {
            XBot::Logger::error("BackEnd does not allow getEpsRegularisation");
            return 0;
        }

    protected:
        ///VIRTUAL METHODS
        /**
         * @brief _log can be used to log extra information
         * @param logger a pointer to a Matlogger
         * @param i an index related to the particular index of the problem
         * @param prefix a prefix before the logged matrices
         */
        virtual void _log(XBot::MatLogger2::Ptr logger, int i, const std::string& prefix){}

        /**
         * @brief _printProblemInformation can be used to print extra information
         */
        virtual void _printProblemInformation(){}

        /**
         * Define a cost function: ||Hx - g||
         */
        Eigen::MatrixXd _H;
        Eigen::VectorXd _g;

        /**
         * Define a set of constraints weighted with A: lA <= Ax <= uA
         */
        Eigen::MatrixXd _A;
        Eigen::VectorXd _lA;
        Eigen::VectorXd _uA;

        /**
         * Define a set of bounds on solution: l <= x <= u
         */
        Eigen::VectorXd _l;
        Eigen::VectorXd _u;

        /**
         * Solution of the QP problem
         */
        Eigen::VectorXd _solution;

        /**
         * @brief _number_of_variables which remain constant during BE existence
         */
        int _number_of_variables;
    };

    }
}

#endif
