#ifndef _WB_SOT_SOLVERS_QP_OASES_TASK_H_
#define _WB_SOT_SOLVERS_QP_OASES_TASK_H_

#include "QPOasesProblem.h"

namespace OpenSoT{
    namespace solvers{

    /**
     * @brief The QPOasesTask class wrapper around QPOasesProblem to easily use Tasks
     */
    class QPOasesTask: public QPOasesProblem
    {
    public:
        /**
         * @brief QPOasesTask constructor with a task and eps regularisation. The problem is initilized and solved.
         * If not solvable an asser is throw. If your bounds are internally to your problem use this constructor.
         * @param task to solve
         * @param eps_regularisation factor
         */
        QPOasesTask(const boost::shared_ptr< Task<Matrix, Vector> >& task, const double eps_regularisation = DEFAULT_EPS_REGULARISATION);

        /**
         * @brief QPOasesTask constructor with a task, bounds and eps regularisation.The problem is initilized and solved.
         * If not solvable an asser is throw. Here bounds are copied explicitely to internal _l and _u.
         * @param task to solve
         * @param bounds bounds to task
         * @param eps_regularisation factor
         */
        QPOasesTask(const boost::shared_ptr< Task<Matrix, Vector> >& task,
                    const boost::shared_ptr< Constraint<Matrix, Vector> >& bounds, const double eps_regularisation = DEFAULT_EPS_REGULARISATION);

        ~QPOasesTask();

        /**
         * @brief solve the internal qp problem
         * @param update_constraints if the problem have to update alone its constraints based
         * on the internal task constraint and bounds. If bounds and constraint are updated outside for the task
         * set it to false.
         * @return true if solved/solvable
         */
        bool solve(bool update_constraints = true);

        /**
         * @brief printProblemInformation couts some information about the problem.
         * @param i, if i = -1 the ID is printed without number:
         * eg:
         *  printProblemInformation();
         *      "PROBLEM 0 ID: com"
         *  printProblemInformation(-1);
         *      "PROBLEM ID: com"
         *  printProblemInformation(2);
         *      "PROBLEM 2 ID: com"
         */
        void printProblemInformation(int i = 0);

        /**
         * @brief velocityControlCostFunction update _H and _g considering a cost function for velovity control:
         * argmin_dq = ||Jdq - v_ref|| = dq'J'Jdq - 2J'dq + v'v
         */
        void velocityControlCostFunction();

        /**
         * @brief getCostFunction return
         * @param H task matrix
         * @param g reference vector
         */
        void getCostFunction(Matrix& H, Vector& g);

        /**
         * @brief getConstraints return
         * @param A constraint matrix
         * @param lA lower constraint vector
         * @param uA upper constraint vector
         */
        void getConstraints(Matrix& A, Vector& lA, Vector& uA);

        /**
         * @brief getBounds return
         * @param l lower bounds vector
         * @param u upper bounds vector
         */
        void getBounds(Vector& l, Vector& u);

        /**
         * @brief getTaskID
         * @return a string with the task id
         */
        std::string getTaskID(){return _task->getTaskID();}

    protected:
        /**
         * @brief _task pointer to task to optimize
         */
        boost::shared_ptr< Task<Matrix, Vector> > _task;

        /**
         * @brief prepareTaskData compute matrices for QPOases
         * @param update_constraints if set to false does not update the constraint matrices
         */
        void prepareTaskData(bool update_constraints_and_bounds = true);
    };

    }
}

#endif
