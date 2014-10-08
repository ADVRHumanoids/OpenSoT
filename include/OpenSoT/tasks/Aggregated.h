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

        private:

            std::list< TaskPtr > _tasks;
            unsigned int _aggregationPolicy;

            void generateAll();
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
