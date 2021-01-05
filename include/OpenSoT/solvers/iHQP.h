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

#define DEFAULT_EPS_REGULARISATION 2E2 //THIS VALUE IS HISTORICALLY USED IN QPOASES

namespace OpenSoT{
class AutoStack;
}

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
         * @brief iHQP constructor of the problem using the AutoStack this constructor permits to use
         * user regularisation feature (inserting it inside the AutoStack)
         * @param stack_of_tasks data structure which will contain tasks, constraints and user defined regularisation
         * NOTICE that the user defined regularisation will be applied to all the stack levels
         * @param eps_regularisation regularisation factor used inside the QP solver (BackEnd)
         * @param be_solver
         * @throw exception if the stack can not be initialized
         */
        iHQP(OpenSoT::AutoStack& stack_of_tasks, const double eps_regularisation = DEFAULT_EPS_REGULARISATION,
             const solver_back_ends be_solver = solver_back_ends::qpOASES);
        iHQP(OpenSoT::AutoStack& stack_of_tasks, const double eps_regularisation,
             const std::vector<solver_back_ends> be_solver);

        /**
         * @brief iHQP constructor of the problem
         * @param stack_of_tasks a vector of tasks
         * @param eps_regularisation regularisation factor
         * @throw exception if the stack can not be initialized
         */
        iHQP(Stack& stack_of_tasks, const double eps_regularisation = DEFAULT_EPS_REGULARISATION,
             const solver_back_ends be_solver = solver_back_ends::qpOASES);
        iHQP(Stack& stack_of_tasks, const double eps_regularisation,
             const std::vector<solver_back_ends> be_solver);

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
        iHQP(Stack& stack_of_tasks,
                    ConstraintPtr bounds,
                    const double eps_regularisation,
                    const std::vector<solver_back_ends> be_solver);

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
        iHQP(Stack& stack_of_tasks,
                    ConstraintPtr bounds,
                    ConstraintPtr globalConstraints,
                    const double eps_regularisation,
                    const std::vector<solver_back_ends> be_solver);


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
         * @param i number of stack to set the option
         * @param opt options for task i
         * @return false if i-th problem does not exists or does not succeed
         */
        bool setOptions(const unsigned int i, const boost::any &opt);

        /**
         * @brief getOptions return the options of the i-th qp problem
         * @param i number of stack to get the option
         * @param opt a data structure which has to be converted to particular structure used by the BackEnd implementation
         * @return false if i-th problem does not exists
         */
        bool getOptions(const unsigned int i, boost::any& opt);

        /**
         * @brief getObjective return the value of the objective function at the optimum for the i-th qp problem
         * @param i number of stack to get the value of the objective function
         * @param val value of the objective function at the optimum
         * @return false if i-th problem does not exists
         */
        bool getObjective(const unsigned int i, double& val);

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

        /**
         * @brief getBackEndName retrieve the name of the solver
         * @return a string with the name of the solver, right now:
         *      "qpOASES"
         *      "OSQP"
         *      "????"
         */
        std::string getBackEndName(const unsigned int i);

        /**
         * @brief setEpsRegularisation OVERWRITES the actual eps regularisation factor of a specific
         * level of the stack
         * @param eps reguralisation factor
         * @param i stack level
         * @return false if eps < 0 or i no in the stack
         */
        bool setEpsRegularisation(const double eps, const unsigned int i);

        /**
         * @brief setEpsRegularisation OVERWRITES the actual eps regularisation factor for all the level of the stack
         * @param eps reguralisation factor
         * @return false if eps < 0
         */
        bool setEpsRegularisation(const double eps);

        /**
         * @brief getBackEnd retrieve the back-end associated to the i-th qp problem
         * @param i priority level
         * @param back_end
         * @return false if the level does not exists
         */
        bool getBackEnd(const unsigned int i, BackEnd::Ptr& back_end);

    protected:
        virtual void _log(XBot::MatLogger2::Ptr logger, const std::string& prefix);

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
        bool prepareSoT(const std::vector<solver_back_ends> be_solver);

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

        //USER REGULARISATION
        Eigen::MatrixXd Hr;
        Eigen::VectorXd gr;

        OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr _regularisation_task;
        //

        MatrixPiler A;
        VectorPiler lA;
        VectorPiler uA;
        
        Eigen::VectorXd l;
        Eigen::VectorXd u;
        
        std::vector<Eigen::MatrixXd> tmp_A;
        std::vector<Eigen::VectorXd> tmp_lA;
        std::vector<Eigen::VectorXd> tmp_uA;


        std::vector<solver_back_ends> _be_solver;

        static const std::string _IHQP_CONSTRAINTS_PLUS_;
        static const std::string _IHQP_CONSTRAINTS_OPTIMALITY_;


    };

    }
}

#endif
