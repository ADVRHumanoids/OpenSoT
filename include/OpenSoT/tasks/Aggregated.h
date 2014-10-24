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

#include <yarp/sig/all.h>
#include <boost/shared_ptr.hpp>
#include <list>


 namespace OpenSoT {
    namespace tasks {

        /**
         * @brief The Aggregated class builds a new Task by piling up simpler tasks
         *        so that A = [W1*A1; W2*A2], b=[W1*alpha1*b1;W2*alpha2*b2]
         */
        class Aggregated: public Task<yarp::sig::Matrix, yarp::sig::Vector> {
        public:
            typedef boost::shared_ptr<Aggregated> Ptr;
        private:

            std::list< TaskPtr > _tasks;
            unsigned int _aggregationPolicy;

            void generateAll();
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
             *      HST_UNKNOWN if at least one of the Hessians in HST_UNKNOWN or all Hessians are HST_SEMIDEF
             *      HST_ZERO if all the Hessians are HST_ZERO
             *      HST_POSDEF if at least one Hessian is HST_POSDEF (and none HST_UNKNOWN)
             *
             * TO DO: Take in consideration also \f$\beta\f$ and \f$\W\f$!
             * @return the Hessian type
             */
            HessianType computeHessianType();
            void checkSizes();
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
                       const yarp::sig::Vector &q);

            ~Aggregated();

            void _update(const yarp::sig::Vector &x);
        };

    }
 }

#endif
