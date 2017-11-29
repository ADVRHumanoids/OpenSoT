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

#ifndef __TASKS_AGGREGATED_H__
#define __TASKS_AGGREGATED_H__

#include <OpenSoT/Task.h>
#include <Eigen/Dense>
#include <boost/shared_ptr.hpp>
#include <list>
#include <OpenSoT/utils/Piler.h>

using namespace OpenSoT::utils;

 namespace OpenSoT {
    namespace tasks {

        /**
         * @brief The Aggregated class builds a new Task by piling up simpler tasks
         *        so that \f$A = [W_1*A_1; W_2*A_2]\f$, \f$b=\lambda*[W_1*\lambda_1*b_1;W_2*\lambda_2*b_2]\f$
         * If the tasks are modified (through their methods, or constraints are added to the tasks),
         * the Aggregated gets updated accordingly (after calling the _update() function), and during the update
         * it will automatically call update() on every single task of which is comprised.
         * It is possible to add constraints to the Aggregated task which are not constraints to the Tasks of which
         * it is comprised. Take a look at getConstraints(), getAggregatedConstraints(), getOwnConstraints() for more infos.
         *
         */
        class Aggregated: public Task<Eigen::MatrixXd, Eigen::VectorXd> {
        public:
            typedef boost::shared_ptr<Aggregated> Ptr;
            typedef MatrixPiler VectorPiler;
        protected:

            std::list< TaskPtr > _tasks;

            std::list< ConstraintPtr > _ownConstraints;
            std::list< ConstraintPtr > _aggregatedConstraints;

            MatrixPiler _tmpA;
            VectorPiler _tmpb;

            unsigned int _aggregationPolicy;

            void generateAll();

            void generateConstraints();

            void generateAggregatedConstraints();

            /**
             * @brief computeHessianType compute the new Hessian type associated to the Aggregated version of the Tasks.
             *
             * In OpenSoT, all the considered Hessians \f$H\f$ are in particular Hermitians and positive semidefinite/definite.
             * Let consider two matrices \f$A\f$ and \f$B\f$ with Hessians \f$H_A = A^TA\f$ and \f$H_B=B^TB\f$, the Aggregated Task
             * \f$C\f$ is computed as:
             * \f$
             *   \begin{equation}
             *     C = \left[A^T \quad B^T\right]^T
             *   \end{equation}
             * \f$
             * and the associated Hessian \f$H_C\f$ is computed as:
             * \f$
             *   \begin{equation}
             *     H_C = \left[A^T \quad B^T\right]\left[A^T \quad B^T\right]^T \\
             *         = A^TA + B^TB = H_A + H_B
             *   \end{equation}
             * \f$
             * In general for an arbitrary number of tasks we can say \f$H_{Agg} = \displaystyle\sum_{i}^{n} H_i\f$. Therefore the problem of
             * determine the new Hessian type for the Aggregated Task can be considered as the problem to determine the Hessian type of the
             * sum of different matrices with known Hessian type.
             * Observation: The sum of any two positive definite matrices of the same size is positive definite. More generally, any
             * nonnegative linear combination of positive semidefinite matrices is positive semidefinite (Roger A. Horn and Charles R. Johnson,
             * Matrix Analysis, Cambridge University Press, 1996, p.398).
             * With these considerations in mind, the method will return:
             *      HST_UNKNOWN if at least one of the Hessians in HST_UNKNOWN
             *      HST_ZERO if all the Hessians are HST_ZERO
             *      HST_POSDEF if at least one Hessian is HST_POSDEF (and none HST_UNKNOWN)
             *      HST_SEMIDEF if all one Hessian is HST_SEMIDEF (and none HST_UNKNOWN)
             *
             * TO DO: Take in consideration also \f$\beta\f$ and \f$\W\f$!
             * @return the Hessian type
             */
            HessianType computeHessianType();

            void checkSizes();

            static const std::string concatenateTaskIds(const std::list<TaskPtr> tasks);

            virtual void _log(XBot::MatLogger::Ptr logger);

        public:
            /**
             * @brief Aggregated
             * @param bounds a std::list of Tasks
             * @param x_size the size of the input vector. Notice this constructor will NOT call
             *               update() on the base tasks
             */
            Aggregated(const std::list< TaskPtr > tasks,
                       const unsigned int x_size);

            /**
             * @brief Aggregated
             * @param task1 a pointer to the first Task to aggregate
             * @param task2 a pointer to the second Task to aggregate
             * @param x_size the size of the input vector. Notice this constructor will NOT call
             *               update() on the base tasks
             */
            Aggregated(TaskPtr task1,
                       TaskPtr task2,
                       const unsigned int x_size);

            /**
             * @brief Aggregated
             * @param bounds a std::list of Tasks
             * @param q the vector of q at which to create the Aggregated task
             *          Notice that by specifying q, the Aggregated will automatically call
             *          update(q) on all tasks he is composed of
             */
            Aggregated(const std::list< TaskPtr > tasks,
                       const Eigen::VectorXd &q);

            ~Aggregated();

            void _update(const Eigen::VectorXd &x);


            /**
             * @brief getConstraints return a reference to the constraint list.
             * Use the standard list methods to add to the constraints list.
             * i.e.:
             *              task.getConstraints().push_back(new_constraint)
             * Notice that the constraints that getConstraints returns are regenerated
             * after each call to the _update() function. A simple check is made before regenerating
             * the constraints, so that if a new constraints gets added to the list before the update,
             * if the constraint is added with a push_back (i.e., at the back of the list), it will be
             * copied to the ownConstraints list of constraints permanently.
             * Notice however that adding constraints in this way is not considered efficient, you should
             * directly add them to the ownConstraints list, by calling,
             * i.e.:
             *              task.getOwnConstraints().push_back(new_constraint)
             * If you want to remove constraints which are not ownConstraints, please remove them
             * from the Tasks of which the Aggregated is composed from.
             * @return a generated list of pointers to constraints
             */

            /**
             * @brief getOwnConstraints return a reference to the Aggregated task own constraints list.
             * The ownConstraints list is used together with all the constraints of every task to generate the
             * final constraints list.
             * Use the standard list methods
             * to add, remove, clear, ... the constraints list.
             * e.g.:
             *              task.getConstraints().push_back(new_constraint)
             * @return the list of constraints for this task
             */
            std::list< ConstraintPtr >& getOwnConstraints() { return _ownConstraints; }

            /**
             * @brief getAggregatedConstraints return a reference to the aggregation of the tasks constraint list.
             * The list cannot be modified, as it is generated starting from all the constraints.
             * If you need to modify this list, you can do it indirectly by modifying the list of constraints
             * of the tasks that compose this Aggregated, and calling _update() on the Aggregated
             * @return the list composed of the constraints for all the tasks included in this Aggregated
             */
            const std::list< ConstraintPtr >& getAggregatedConstraints() { return _aggregatedConstraints; }

            const std::list< TaskPtr >& getTaskList() { return _tasks; }

            /**
             * @brief setLambda set the lambda to ALL the aggregated tasks to the same value lambda.
             * The lambda associated to the Aggregate and the lambda associated to the tasks are different if a
             * Aggregated.setLambda(lambda) is not called.
             * @param lambda a value for all the tasks in the aggregate
             */
            void setLambda(double lambda);
              
            static bool isAggregated(OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr task);
        };

    }
 }

#endif
