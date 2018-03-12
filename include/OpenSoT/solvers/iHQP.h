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
#include <OpenSoT/solvers/BackEndFactory.h>
#include <OpenSoT/utils/Piler.h>

using namespace OpenSoT::utils;

namespace OpenSoT{
    namespace solvers{

    /**
     * @brief The iHQP class implement a solver that accept a Stack of Tasks with Bounds and Constraints
     */
    class iHQP: public Solver<Eigen::MatrixXd, Eigen::VectorXd>
    {
    public:
    typedef boost::shared_ptr<iHQP> Ptr;
    typedef MatrixPiler VectorPiler;
        /**
         * @brief iHQP constructor of the problem
         * @param stack_of_tasks a vector of tasks
         * @param eps_regularisation regularisation factor
         * @throw exception if the stack can not be initialized
         */
        iHQP(Stack& stack_of_tasks, const double eps_regularisation = DEFAULT_EPS_REGULARISATION,
             const solver_back_ends be_solver = solver_back_ends::qpOASES);

        /**
         * @brief iHQP constructor of the problem
         * @param stack_of_tasks a vector of tasks
         * @param bounds a vector of bounds passed to all the stacks
         * @param eps_regularisation regularisation factor
         * @throw exception if the stack can not be initialized
         */
        iHQP(Stack& stack_of_tasks,
                    ConstraintPtr bounds,
                    const double eps_regularisation = DEFAULT_EPS_REGULARISATION,
                    const solver_back_ends be_solver = solver_back_ends::qpOASES);

        /**
         * @brief iHQP constructor of the problem
         * @param stack_of_tasks a vector of tasks
         * @param bounds a vector of bounds passed to all the stacks
         * @param globalConstraints a vector of constraints passed to all the stacks
         * @param eps_regularisation regularisation factor
         * @throw exception if the stack can not be initialized
         */
        iHQP(Stack& stack_of_tasks,
                    ConstraintPtr bounds,
                    ConstraintPtr globalConstraints,
                    const double eps_regularisation = DEFAULT_EPS_REGULARISATION,
                    const solver_back_ends be_solver = solver_back_ends::qpOASES);


        ~iHQP(){}

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
        bool setOptions(const unsigned int i, const boost::any &opt);

        /**
         * @brief getOptions
         * @param i
         * @param opt
         * @return
         */
        bool getOptions(const unsigned int i, boost::any& opt);

        /**
         * @brief setActiveStack select a stack to do not solve
         * @param i stack index
         * @param flag true or flase
         */
        void setActiveStack(const unsigned int i, const bool flag);

        /**
         * @brief activateAllStacks activate all stacks
         */
        void activateAllStacks();

    protected:
        virtual void _log(XBot::MatLogger::Ptr logger);

        vector <OpenSoT::constraints::Aggregated> constraints_task;
        
        /**
         * @brief _qp_stack_of_tasks vector of QPOases Problem
         */
        vector <BackEnd::Ptr> _qp_stack_of_tasks;

        vector<bool> _active_stacks;

        /**
         * @brief _epsRegularisation regularisation factor for dumped least squares
         */
        double _epsRegularisation;

        /**
         * @brief prepareSoT initialize the complete stack
         * @return true if stack is correctly initialized
         */
        bool prepareSoT(const solver_back_ends be_solver);

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
        void computeOptimalityConstraint(const TaskPtr& task, BackEnd::Ptr& problem,
                                         Eigen::MatrixXd& A,
                                         Eigen::VectorXd& lA, Eigen::VectorXd& uA);



        Eigen::MatrixXd H;
        Eigen::VectorXd g;

        MatrixPiler A;
        VectorPiler lA;
        VectorPiler uA;
        
        Eigen::VectorXd l;
        Eigen::VectorXd u;
        
        std::vector<Eigen::MatrixXd> tmp_A;
        std::vector<Eigen::VectorXd> tmp_lA;
        std::vector<Eigen::VectorXd> tmp_uA;



    };

    }
}

#endif
