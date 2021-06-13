#ifndef __SOLVERS_HCOD__
#define __SOLVERS__HCOD__

#include <OpenSoT/Solver.h>
#include <Eigen/Dense>
#include <OpenSoT/utils/AutoStack.h>
#include <OpenSoT/tasks/Aggregated.h>
#include <soth/HCOD.hpp>

namespace OpenSoT{
    namespace solvers{
        /**
         * @brief The HCOD class implements the front-end for the hcod solver in
         * https://github.com/stack-of-tasks/soth
         */
        class HCOD: public OpenSoT::Solver<Eigen::MatrixXd, Eigen::VectorXd>
        {
            public:
                typedef std::shared_ptr<HCOD> Ptr;
                typedef MatrixPiler VectorPiler;

                /**
                 * @brief HCOD
                 * @param stack_of_tasks
                 * @param damping factor (like regularization)
                 */
                HCOD(OpenSoT::AutoStack& stack_of_tasks, const double damping);

                /**
                 * @brief HCOD
                 * @param stack_of_tasks vector of prioritized tasks
                 * @param bounds vector of constraints
                 * @param damping factor (like regularization)
                 */
                HCOD(Stack& stack_of_tasks, ConstraintPtr bounds, const double damping);

                /**
                  * brief ~HCOD destructor
                  */
                ~HCOD(){}

                /**
                 * @brief solve a stack of tasks
                 * @param solution vector
                 * @return true if all the stack is solved
                 */
                bool solve(Eigen::VectorXd& solution);

                /**
                 * @brief getInternalSolver
                 * @return pointer to internal hcod solver
                 */
                std::shared_ptr<soth::HCOD> getInternalSolver() {return _hcod;}

            private:
                /**
                 * @brief _hcod internal solver
                 */
                std::shared_ptr<soth::HCOD> _hcod;

                /**
                 * @brief _vector_bounds internal vector to store constraints
                 */
                std::vector<soth::VectorBound> _vector_bounds;

                /**
                 * @brief _vector_J internal vector to store constraints and tasks matrices
                 * NOTE: in our implementation ALL the constraints are considered at first
                 * priority level. The tasks follow the task priorities.
                 */
                std::vector<Eigen::MatrixXd> _vector_J;

                /**
                 * @brief _VARS number of variables
                 */
                int _VARS;

                /**
                 * @brief _CL = 1 if constraints are present
                 */
                int _CL;

                /**
                 * @brief init initialize internal hcod solver
                 * @param damping factor
                 */
                void init(const double damping);

                /**
                 * @brief copy_bounds copies _bounds into _vector_bounds and _vector_J
                 */
                void copy_bounds();

                /**
                 * @brief copy_tasks copies _tasks into _vector_bounds and _vector_J
                 */
                void copy_tasks();

                /**
                 * @brief _A matrix to pile constraints matrices
                 */
                MatrixPiler _A;

                /**
                 * @brief _lA to pile bounds
                 */
                VectorPiler _lA;

                /**
                 * @brief _uA to pile bounds
                 */
                VectorPiler _uA;

                /**
                 * @brief _I Identity to be used for bounds
                 */
                Eigen::MatrixXd _I;
        };
    }
}

#endif
