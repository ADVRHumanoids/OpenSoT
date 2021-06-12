#ifndef __SOLVERS_HCOD__
#define __SOLVERS__HCOD__

#include <OpenSoT/Solver.h>
#include <Eigen/Dense>
#include <OpenSoT/utils/AutoStack.h>
#include <OpenSoT/tasks/Aggregated.h>
#include <soth/HCOD.hpp>

namespace OpenSoT{
    namespace solvers{
        class HCOD: public OpenSoT::Solver<Eigen::MatrixXd, Eigen::VectorXd>
        {
            public:
                typedef std::shared_ptr<HCOD> Ptr;
                typedef MatrixPiler VectorPiler;

                HCOD(OpenSoT::AutoStack& stack_of_tasks, const double damping);
                ~HCOD(){}

                /**
                 * @brief solve a stack of tasks
                 * @param solution vector
                 * @return true if all the stack is solved
                 */
                bool solve(Eigen::VectorXd& solution);


                std::shared_ptr<soth::HCOD> getInternalSolver() {return _hcod;}

            private:
                std::shared_ptr<soth::HCOD> _hcod;
                std::vector<soth::VectorBound> _vector_bounds;
                std::vector<Eigen::MatrixXd> _vector_J;

                int _VARS;
                int _CL;

                /**
                 * @brief copy_bounds copy _bounds into _vector_bounds and _vector_J
                 */
                void copy_bounds();

                void copy_tasks();

                //These are to handle constraints
                MatrixPiler _A;
                VectorPiler _lA;
                VectorPiler _uA;
                Eigen::MatrixXd _I;
        };
    }
}

#endif
