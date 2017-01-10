/*
 * Copyright (C) 2014 Walkman
 * Author: Enrico Mingo, Alessio Rocchi
 * email:  enrico.mingo@iit.it, alessio.rocchi@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU Lesser General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#ifndef _WB_SOT_SOLVERS_QP_OASES_H_
#define _WB_SOT_SOLVERS_QP_OASES_H_

#include <vector>
#include <iostream>
#include <boost/shared_ptr.hpp>
#include <OpenSoT/Task.h>
#include <OpenSoT/Solver.h>
#include <OpenSoT/constraints/Aggregated.h>
#include "QPOasesProblem.h"


namespace qpOASES {
    class SQProblem;
    class Options;
    class Bounds;
    class Constraints;
}


namespace OpenSoT{
    namespace solvers{

    /**
     * @brief The QPOases_sot class implement a solver that accept a Stack of Tasks with Bounds and Constraints
     */
    class QPOases_sot: public Solver<Eigen::MatrixXd, Eigen::VectorXd>
    {
    public:
	typedef boost::shared_ptr<QPOases_sot> Ptr;
        /**
         * @brief QPOases_sot constructor of the problem
         * @param stack_of_tasks a vector of tasks
         * @param eps_regularisation regularisation factor
         * @throw exception if the stack can not be initialized
         */
        QPOases_sot(Stack& stack_of_tasks, const double eps_regularisation = DEFAULT_EPS_REGULARISATION);

        /**
         * @brief QPOases_sot constructor of the problem
         * @param stack_of_tasks a vector of tasks
         * @param bounds a vector of bounds passed to all the stacks
         * @param eps_regularisation regularisation factor
         * @throw exception if the stack can not be initialized
         */
        QPOases_sot(Stack& stack_of_tasks,
                    ConstraintPtr bounds,
                    const double eps_regularisation = DEFAULT_EPS_REGULARISATION);

        /**
         * @brief QPOases_sot constructor of the problem
         * @param stack_of_tasks a vector of tasks
         * @param bounds a vector of bounds passed to all the stacks
         * @param globalConstraints a vector of constraints passed to all the stacks
         * @param eps_regularisation regularisation factor
         * @throw exception if the stack can not be initialized
         */
        QPOases_sot(Stack& stack_of_tasks,
                    ConstraintPtr bounds,
                    ConstraintPtr globalConstraints,
                    const double eps_regularisation = DEFAULT_EPS_REGULARISATION);


        ~QPOases_sot(){}

        /**
         * @brief solve a stack of tasks
         * @param solution vector
         * @return true if all the stack is solved
         */
        bool solve(Eigen::VectorXd& solution);

        /**
         * @brief getNumberOfTasks
         * @return lenght of the stack
         */
        unsigned int getNumberOfTasks(){return _qp_stack_of_tasks.size();}

        /**
         * @brief setOptions set option to a particular task
         * @param i number of task to set the option
         * @param opt options for task i
         * @return true if succeed
         */
        bool setOptions(const unsigned int i, const qpOASES::Options &opt);

        /**
         * @brief getOptions
         * @param i
         * @param opt
         * @return
         */
        bool getOptions(const unsigned int i, qpOASES::Options& opt);

    protected:
        /**
         * @brief _qp_stack_of_tasks vector of QPOases Problem
         */
        vector <QPOasesProblem> _qp_stack_of_tasks;

        /**
         * @brief _epsRegularisation regularisation factor for dumped least squares
         */
        double _epsRegularisation;

        /**
         * @brief prepareSoT initialize the complete stack
         * @return true if stack is correctly initialized
         */
        bool prepareSoT();

        /**
         * @brief computeCostFunction compute a cost function for velocity control:
         *          F = ||Jdq - v||
         * @param task to get Jacobian and reference
         * @param H Hessian matrix computed as J'J
         * @param g reference vector computed as J'v
         */
        void computeCostFunction(const TaskPtr& task, Eigen::MatrixXd& H, Eigen::VectorXd& g);

        /**
         * @brief computeOptimalityConstraint compute optimality constraint for velocity control:
         *      Jj*dqj = Jj*dqi
         * @param task to get Jacobian of the previous task
         * @param problem to get solution of the previous task
         * @param A constraint matrix
         * @param lA lower bounds
         * @param uA upper bounds
         */
        void computeOptimalityConstraint(const TaskPtr& task, QPOasesProblem& problem,
                                                Eigen::MatrixXd& A, Eigen::VectorXd& lA, Eigen::VectorXd& uA);

        inline void pile(Eigen::MatrixXd& A, const Eigen::MatrixXd& B)
        {
            A.conservativeResize(A.rows()+B.rows(), A.cols());
            A.block(A.rows()-B.rows(),0,B.rows(),A.cols())<<B;
        }

        inline void pile(Eigen::VectorXd &a, const Eigen::VectorXd &b)
        {
            a.conservativeResize(a.rows()+b.rows());
            a.segment(a.rows()-b.rows(),b.rows())<<b;
        }

    };

    }
}

#endif
