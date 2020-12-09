#ifndef _L1_HQP_
#define _L1HQP_

#include <OpenSoT/Solver.h>
#include <OpenSoT/solvers/BackEndFactory.h>
#include <OpenSoT/utils/Affine.h>
#include <OpenSoT/constraints/GenericConstraint.h>
#include <OpenSoT/tasks/GenericLPTask.h>

#define DEFAULT_EPS_REGULARISATION 2E2 //THIS VALUE IS HISTORICALLY USED IN QPOASES

namespace OpenSoT{
class AutoStack;
}

namespace OpenSoT {
namespace solvers {

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
             * @brief getInternalProblem just for debugging
             * @return
             */
            const boost::shared_ptr<AutoStack>& getInternalProblem(){ return _internal_stack;}

        private:
            double _epsRegularisation;

            OpenSoT::AutoStack& _stack_of_tasks;
            boost::shared_ptr<OptvarHelper> _opt;
            /**
             * @brief _linear_gains vector of gains to keep priorities from the highest [0] to the lowest [l]
             */
            std::map<std::string, Eigen::VectorXd> _linear_gains;

            OpenSoT::constraints::GenericConstraint::Ptr _constraint;
            std::map<std::string, OpenSoT::tasks::GenericLPTask::Ptr> _lp_tasks;


            boost::shared_ptr<AutoStack> _internal_stack;


            void creates_problem_variables();
            void creates_tasks();
            void creates_internal_problem();


    };

}
}

#endif
