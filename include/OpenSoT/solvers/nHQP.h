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

#ifndef __SOT_SOLVERS_NULLSPACE_HQP_H__
#define __SOT_SOLVERS_NULLSPACE_HQP_H__

#include <vector>
#include <iostream>

#include <boost/shared_ptr.hpp>

#include <eigen3/Eigen/SVD>

#include <OpenSoT/Task.h>
#include <OpenSoT/Solver.h>
#include <OpenSoT/constraints/Aggregated.h>
#include <OpenSoT/solvers/BackEndFactory.h>
#include <OpenSoT/utils/Piler.h>

namespace OpenSoT { namespace solvers {
/**
     * @brief Hierarchical QP solver based on nullspace basis optimization.
     *
     * Layer #0: min |A0*x-b0| -> let N0 = Null(A0) be a basis of the nullspace of A0, and x0_opt = sol0 the solution
     *
     * Layer #1: min |A1*x-b1|. Optimize on the nullspace coordinates (x1) by setting: x = sol0 + N0*x1.
     *   This results in A1 -> A1*N0,  b1 -> b1 - A1*sol0.
     *   Then, let N1 be N0*Null(A1*N0), sol1 = sol0 + N0*x1_opt
     *
     * Layer #K: min |AK*x-bK| -> x = solK-1 + NK-1*xK
     *   AK -> AK*NK-1, bK -> bK - AK*solK-1
     *   solK = solK-1 + NK-1*xK_opt
     *   NK = NK-1 * Null(AK*NK-1)
     *
     * Notice how each layer optimizes only over the remaining dofs after higher priority tasks
     * have been optimized. Hence, the size of QP probles decreases along the hierarchy.
     *
     * Limitations:
     *  - no support for equality constraints
     *  - no support for local constraints
     *
     * TODO: implement equality constraints by considering the nullspace Neq = Null(Aeq).
     * All optimizations will then take place in the null space of equality constraints.
     */
    class nHQP: public Solver<Eigen::MatrixXd, Eigen::VectorXd>
    {
        
    public:
        
        // Shared pointer typedef
        typedef boost::shared_ptr<nHQP> Ptr;
        
        // Constructor
        nHQP(Stack& stack_of_tasks,
             ConstraintPtr bounds,
             const double eps_regularisation,
             const solver_back_ends be_solver = solver_back_ends::qpOASES);

        // Destructor
        virtual ~nHQP() override;

        // Solve implementation
        virtual bool solve(Eigen::VectorXd& solution) override;


    private:
        
        /**
         * @brief The TaskData class is a helper class containing all that is needed to setup,
         * regularize, and solve the i-th QP problem.
         */
        class TaskData
        {
            
        public:
            
            TaskData(int num_free_vars,
                     TaskPtr task,
                     ConstraintPtr constraint,
                     BackEnd::Ptr back_end);
            
            void compute_cost(const Eigen::MatrixXd * N, const Eigen::VectorXd& q0);
            void compute_contraints(const Eigen::MatrixXd * N, const Eigen::VectorXd& q0);
            bool compute_nullspace();
            
            const Eigen::MatrixXd& get_nullspace() const;
            bool update_and_solve();
            const Eigen::VectorXd& get_solution() const;
            
        private:
            
            const int num_vars;
            const int size;
            
            // this task
            TaskPtr task;

            // global constraints
            ConstraintPtr constraints;
            
            // nullspace of AN (used by next task)
            Eigen::MatrixXd N;

            // A matrix of this task (A projected onto previous tasks nullspace)
            Eigen::MatrixXd AN;

            // b vector for this task
            Eigen::VectorXd b0;
            
            // quadratic cost matrices
            Eigen::MatrixXd H;
            Eigen::VectorXd g;

            // inequality constraints (including bounds for i = 1, 2, ...)
            utils::MatrixPiler Aineq;

            // inequality bounds
            utils::MatrixPiler lb, ub;

            // simple bounds (only i = 0)
            Eigen::VectorXd lb_bound, ub_bound;
            
            // svd computation class
            Eigen::BDCSVD<Eigen::MatrixXd> svd;

            // backend for solving the QP
            BackEnd::Ptr back_end;

            /**
             * @brief Perform SVD-based regularization of AN and b0 as follows:
             *  - components of b0 along A's singular vectors with low singular value are deflated
             *  - singular values of A under the threshold are inflated
             *
             * @param threshold
             */
            void regularize_A_b(double threshold);

        };

        
        virtual void _log(XBot::MatLogger::Ptr logger, const std::string& prefix) override;

        // vector of previous task nullspaces (first elem is nx-by-nx identity)
        std::vector<Eigen::MatrixXd> _cumulated_nullspace;

        // task data for all layers
        std::vector<TaskData> _data_struct;

        // to store solution
        Eigen::VectorXd _solution;


    };

    
} }

#endif





