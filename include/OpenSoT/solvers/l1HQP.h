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
     /**
     * @brief The priority_constraint class implements the priority constraint:
     *
     *          c1't1 >= c2't2
     */
    class priority_constraint: public Constraint<Eigen::MatrixXd, Eigen::VectorXd>
    {
    public:
        typedef boost::shared_ptr<priority_constraint> Ptr;
        priority_constraint(const std::string& id,
                            const OpenSoT::tasks::GenericLPTask::Ptr high_priority_task,
                            const OpenSoT::tasks::GenericLPTask::Ptr low_priority_task);
        void update(const Eigen::VectorXd& x);
    private:
        boost::weak_ptr<OpenSoT::tasks::GenericLPTask> _high_task;
        boost::weak_ptr<OpenSoT::tasks::GenericLPTask> _low_task;

        Eigen::MatrixXd _ones;
    };

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
         *              -Mt <= Ax - b <= Mt
         *                0 <= t <= 1
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

        Eigen::VectorXd o, inf, ones;
        Eigen::MatrixXd O;
        double M = 10.; //This is for the Big-M constraint
    };

    class l1HQP: public Solver<Eigen::MatrixXd, Eigen::VectorXd>
    {
        public:
            typedef boost::shared_ptr<l1HQP> Ptr;

            /**
             * @brief l1HQP oncstructor
             * @param stack_of_tasks stack
             * @param eps_regularisation refers to the L2 regularisation
             * @param be_solver internal qp solver
             */
            l1HQP(OpenSoT::AutoStack& stack_of_tasks, const double eps_regularisation = DEFAULT_EPS_REGULARISATION,
             const solver_back_ends be_solver = solver_back_ends::qpOASES);

            bool solve(Eigen::VectorXd& solution);

            /**
             * @brief getFirstSlackIndex
             * @return index to first (internal) slack variable, -1 if slack variables are not present
             */
            unsigned int getFirstSlackIndex(){ return _first_slack_index;}

            void getBackEnd(BackEnd::Ptr& back_end);

            /**
             * @brief getInternalProblem(), getConstraints(), getHardConstraints(), getTasks() and
             * getPriorityConstraints() are ONLY for debugging
             * @return
             */
            const boost::shared_ptr<AutoStack>& getInternalProblem(){ return _internal_stack;}
            const std::map<std::string, task_to_constraint_helper::Ptr>& getConstraints(){ return _constraints; }
            const constraint_helper::Ptr& getHardConstraints(){return _constraints2; }
            const std::map<std::string, OpenSoT::tasks::GenericLPTask::Ptr>& getTasks(){ return _lp_tasks; }
            const std::vector<priority_constraint::Ptr>& getPriorityConstraints(){ return _priority_constraints; }


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
             * @brief _linear_gains vector of gains
             */
            std::map<std::string, Eigen::VectorXd> _linear_gains;

            /**
             * @brief _task_id_priority_order vector to keep priorities order from the highest [0] to the lowest [l]
             */
            std::vector<std::string> _task_id_priority_order;
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
            Eigen::MatrixXd _H;
            OpenSoT::HessianType _hessian_type;

            /**
             * @brief _priority_constraints implements constraints in the form:
             *
             *          c1't1 <= c2't2
             *
             * with c1't1 task with higher priority wrt c2't2
             */
            std::vector<priority_constraint::Ptr> _priority_constraints;

            unsigned int _first_slack_index;


    };

}
}

#endif
