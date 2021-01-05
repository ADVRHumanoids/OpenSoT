/*
 * Copyright (C) 2014 Walkman
 * Author: Alessio Rocchi
 * email:  alessio.rocchi@iit.it
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

#ifndef __BOUNDS_TASKTOCONSTRAINT_H__
#define __BOUNDS_TASKTOCONSTRAINT_H__

#include <OpenSoT/constraints/BilateralConstraint.h>
#include <OpenSoT/Task.h>

#include <list>


 namespace OpenSoT {
    namespace constraints {

        /**
         * @brief The TaskToConstraint class transforms a task into an inequality constraint
         * which bounds the error in between err_lb and err_ub
         *
         * BilateralConstraint:
         *   \f$ b + err_lb <= A*x <= b + err_ub \f$
         */
        class TaskToConstraint: public BilateralConstraint {
        public:
            typedef boost::shared_ptr< OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd> > TaskPtr;
            typedef boost::shared_ptr< OpenSoT::constraints::TaskToConstraint> Ptr;

        private:
            
            TaskPtr _task;
            
            Eigen::VectorXd _err_lb, _err_ub;

        public:

            /**
             * @brief TaskToConstraint creates an equality TaskToConstraint from a task
             * Calling update on the constraint will also update the task
             * @param task
             */
            TaskToConstraint(TaskPtr task);
            
             /**
             * @brief TaskToConstraint creates a TaskToConstraint from a task
             * Calling update on the constraint will also update the task
             * @param task
             */
            TaskToConstraint(TaskPtr task, 
                             const Eigen::VectorXd& err_lb, 
                             const Eigen::VectorXd& err_ub);

            /**
             * @brief update updates the adapted task and the adapter constraint
             * @param q
             */
            void update(const Eigen::VectorXd &q);

        protected:

            void generateAll();

            /**
             * @brief _log can be used to log internal Constraint variables
             * @param logger a shared pointer to a MatLogger
             */
            virtual void _log(XBot::MatLogger2::Ptr logger)
            {
                _task->log(logger);
            }
        };
    }
 }

#endif
