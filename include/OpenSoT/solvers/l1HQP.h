#ifndef _L1_HQP_
#define _L1HQP_

#include <OpenSoT/Solver.h>
#include <OpenSoT/solvers/BackEndFactory.h>
#include <OpenSoT/utils/Affine.h>
#include <OpenSoT/constraints/GenericConstraint.h>
#include <OpenSoT/tasks/GenericLPTask.h>
#include <OpenSoT/tasks/Aggregated.h>
#include <OpenSoT/utils/Piler.h>
#include <OpenSoT/constraints/Aggregated.h>
#include <OpenSoT/solvers/BackEnd.h>

#define DEFAULT_EPS_REGULARISATION 2E2 //THIS VALUE IS HISTORICALLY USED IN QPOASES

namespace OpenSoT{
class AutoStack;
}

namespace OpenSoT {
namespace solvers {
    class constraint_helper: public Constraint<Eigen::MatrixXd, Eigen::VectorXd>
    {
    public:
        typedef boost::shared_ptr<constraint_helper> Ptr;

        constraint_helper(std::string id, OpenSoT::constraints::Aggregated::ConstraintPtr constraints,
                          const AffineHelper& x);

        void update(const Eigen::VectorXd& x);
    private:
        OpenSoT::constraints::Aggregated::ConstraintPtr _constraints;
        AffineHelper _constraint;
        AffineHelper _x;
        Eigen::MatrixXd I;

        OpenSoT::utils::MatrixPiler _A;
        OpenSoT::utils::MatrixPiler _b_lower;
        OpenSoT::utils::MatrixPiler _b_upper;

    };

    class task_to_constraint_helper: public Constraint<Eigen::MatrixXd, Eigen::VectorXd>
    {
    public:
        typedef boost::shared_ptr<task_to_constraint_helper> Ptr;
        /**
         * @brief task_to_constraint_helper is an helper class to transform tasks from the form:
         *              Ax - b = 0
         * to:
         *              -t <= Ax - b <= t
         * @param id internal name of the task
         * @param task a Task pointer
         * @param x task variable
         * @param t extra variable
         */
        task_to_constraint_helper(std::string id, OpenSoT::tasks::Aggregated::TaskPtr& task,
                                  const AffineHelper& x, const AffineHelper& t);

        void update(const Eigen::VectorXd& x);
    private:
        OpenSoT::tasks::Aggregated::TaskPtr& _task;
        AffineHelper _constraint;
        AffineHelper _x;
        AffineHelper _t;

        OpenSoT::utils::MatrixPiler _II;
        OpenSoT::utils::MatrixPiler _AA;
        OpenSoT::utils::MatrixPiler _bb;

        Eigen::VectorXd o;
        Eigen::MatrixXd O;
    };

    class l1HQP: public Solver<Eigen::MatrixXd, Eigen::VectorXd>
    {
        public:
            typedef boost::shared_ptr<l1HQP> Ptr;

            l1HQP(OpenSoT::AutoStack& stack_of_tasks, const double eps_regularisation = DEFAULT_EPS_REGULARISATION,
             const solver_back_ends be_solver = solver_back_ends::qpOASES);

            bool solve(Eigen::VectorXd& solution);

            /**
             * @brief getPriorityGains this could be used to change on the fly priorities!
             * @return map<level, LPTask>, level names are "t0", "t1", ...
             */
            std::map<std::string, OpenSoT::tasks::GenericLPTask::Ptr>& getTasks(){ return _lp_tasks; }

            /**
             * @brief getInternalProblem(), getConstraints() and getHardConstraints() are just for debugging
             * @return
             */
            const boost::shared_ptr<AutoStack>& getInternalProblem(){ return _internal_stack;}
            const std::map<std::string, task_to_constraint_helper::Ptr>& getConstraints(){ return _constraints; }
            const constraint_helper::Ptr& getHardConstraints(){return _constraints2; }

            unsigned int getVariableSize(){ return _opt->getSize();}


            /**
             * @brief getInternalVariable
             * @param var names are "t1", "t2", ...
             * @param value vector of values
             * @return false if asked internal variable does not exists
             */
            bool getInternalVariable(const std::string& var, Eigen::VectorXd& value);



        private:
            double _epsRegularisation;

            OpenSoT::AutoStack& _stack_of_tasks;
            boost::shared_ptr<OptvarHelper> _opt;
            /**
             * @brief _linear_gains vector of gains to keep priorities from the highest [0] to the lowest [l]
             */
            std::map<std::string, Eigen::VectorXd> _linear_gains;

            std::map<std::string, OpenSoT::tasks::GenericLPTask::Ptr> _lp_tasks;

            /**
             * @brief _constraints is a map containing all the tasks which became constraints in the form:
             *
             *              -t <= Ax - b <= t
             */
            std::map<std::string, task_to_constraint_helper::Ptr> _constraints;


            boost::shared_ptr<AutoStack> _internal_stack;

            /**
             * @brief _constraints2 contains all the constraints coming from the problem which remains in the form:
             *
             *          l <= Cx - d <= u
             */
            constraint_helper::Ptr _constraints2;


            void creates_problem_variables();
            void creates_tasks();
            void creates_internal_problem();
            void creates_constraints();
            bool creates_solver(const solver_back_ends);

            OpenSoT::solvers::BackEnd::Ptr _solver;

            Eigen::VectorXd _internal_solution;


    };

}
}

#endif
