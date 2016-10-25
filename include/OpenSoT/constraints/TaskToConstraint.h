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
         * @brief The TaskToConstraint class transforms a task into an equality constraint:
         *        \f$ A*x = b \f$
         * BilateralConstraint:
         *   \f$ b <= A*x <= b \f$
         */
        class TaskToConstraint: public BilateralConstraint {
        public:
            typedef boost::shared_ptr< OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd> > TaskPtr;
            typedef boost::shared_ptr< OpenSoT::constraints::TaskToConstraint> Ptr;

        private:
            TaskPtr _task;

        public:

            /**
             * @brief TaskToConstraint creates a TaskToConstraint from a task
             * Calling update on the constraint will also update the task
             * @param task
             */
            TaskToConstraint(TaskPtr task);

            /**
             * @brief update updates the adapted task and the adapter constraint
             * @param q
             */
            void update(const Eigen::VectorXd &q);

        protected:

            void generateAll();
        };
    }
 }

#endif
